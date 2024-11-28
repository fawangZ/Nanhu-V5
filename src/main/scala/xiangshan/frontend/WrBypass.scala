/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/
package xiangshan.frontend

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import utils._
import xs.utils._
import xs.utils.perf._
import xs.utils.sram._
import xiangshan.cache.mmu.CAMTemplate

class WrBypass[T <: Data](gen: T, val numEntries: Int, val idxWidth: Int,
  val numWays: Int = 1, val tagWidth: Int = 0, val extraPort: Option[Boolean] = None)(implicit p: Parameters) extends XSModule {
  require(numEntries >= 0)
  require(idxWidth > 0)
  require(numWays >= 1)
  require(tagWidth >= 0)
  def hasTag = tagWidth > 0
  def multipleWays = numWays > 1
  val io = IO(new Bundle {
    val wen = Input(Bool())
    val write_idx = Input(UInt(idxWidth.W))
    val write_tag = if (hasTag) Some(Input(UInt(tagWidth.W))) else None
    val write_data = Input(Vec(numWays, gen))
    val write_way_mask = if (multipleWays) Some(Input(Vec(numWays, Bool()))) else None

    val conflict_valid = if(extraPort.isDefined) Some(Input(Bool())) else None
    val conflict_write_data = if(extraPort.isDefined) Some(Input(Vec(numWays, gen))) else None
    val conflict_way_mask = if(extraPort.isDefined) Some(Input(UInt(numBr.W))) else None

    val hit = Output(Bool())
    val hit_data = Vec(numWays, Valid(gen))
    val has_conflict = if(extraPort.isDefined) Some(Output(Bool())) else None
    val update_idx = if(extraPort.isDefined) Some(Output(UInt(idxWidth.W))) else None
    val update_data = if(extraPort.isDefined) Some(Output(Vec(numWays, gen))) else None
    val update_way_mask = if(extraPort.isDefined) Some(Output(UInt(numBr.W))) else None

    val conflict_clean = if(extraPort.isDefined) Some(Input(Bool())) else None
  })

  class Idx_Tag extends Bundle {
    val idx = UInt(idxWidth.W)
    val tag = if (hasTag) Some(UInt(tagWidth.W)) else None
    def apply(idx: UInt, tag: UInt) = {
      this.idx := idx
      this.tag.map(_ := tag)
    }
  }

  val idx_tag_cam = Module(new IndexableCAMTemplate(new Idx_Tag, numEntries, 1, isIndexable = extraPort.isDefined))
  val data_mem = Mem(numEntries, Vec(numWays, gen))

  val valids = RegInit(0.U.asTypeOf(Vec(numEntries, Vec(numWays, Bool()))))
  val ever_written = RegInit(0.U.asTypeOf(Vec(numEntries, Bool())))


  idx_tag_cam.io.r.req(0)(io.write_idx, io.write_tag.getOrElse(0.U))
  val hits_oh = idx_tag_cam.io.r.resp(0).zip(ever_written).map {case (h, ew) => h && ew}
  val hit_idx = OHToUInt(hits_oh)
  val hit = hits_oh.reduce(_||_)

  io.hit := hit
  for (i <- 0 until numWays) {
    io.hit_data(i).valid := Mux1H(hits_oh, valids)(i)
    io.hit_data(i).bits  := data_mem.read(hit_idx)(i)
  }

  // Replacer
  // Because data_mem can only write to one index
  // Implementing a per-way replacer is meaningless
  // So here use one replacer for all ways
  val replacer = ReplacementPolicy.fromString("plru", numEntries) // numEntries in total
  val replacer_touch_ways = Wire(Vec(1, Valid(UInt(log2Ceil(numEntries).W)))) // One index at a time
  val enq_idx = replacer.way
  val full_mask = Fill(numWays, 1.U(1.W)).asTypeOf(Vec(numWays, Bool()))
  val update_way_mask = io.write_way_mask.getOrElse(full_mask)

  // write data on every request
  when (io.wen) {
    val data_write_idx = Mux(hit, hit_idx, enq_idx)
    data_mem.write(data_write_idx, io.write_data, update_way_mask)
  }
  replacer_touch_ways(0).valid := io.wen
  replacer_touch_ways(0).bits := Mux(hit, hit_idx, enq_idx)
  replacer.access(replacer_touch_ways)

  // update valids
  for (i <- 0 until numWays) {
    when (io.wen) {
      when (hit) {
        when (update_way_mask(i)) {
          valids(hit_idx)(i) := true.B
        }
      }.otherwise {
        ever_written(enq_idx) := true.B
        valids(enq_idx)(i) := false.B
        when (update_way_mask(i)) {
          valids(enq_idx)(i) := true.B
        }
      }
    }
  }

  val enq_en = io.wen && !hit
  idx_tag_cam.io.w.valid := enq_en
  idx_tag_cam.io.w.bits.index := enq_idx
  idx_tag_cam.io.w.bits.data(io.write_idx, io.write_tag.getOrElse(0.U))

  //Extra ports are used to handle dual port read/write conflicts
  if (extraPort.isDefined) {
    val conflict_flags = RegInit(0.U.asTypeOf(Vec(numEntries, Bool())))
    val conflict_way_mask = RegInit(0.U.asTypeOf(io.conflict_way_mask.get))
    val conflict_data = RegInit(VecInit(Seq.tabulate(numWays)( i => 0.U.asTypeOf(gen))))
    val conflict_idx = OHToUInt(conflict_flags)

    idx_tag_cam.io.ridx.get := conflict_idx

    when (io.wen && io.conflict_valid.getOrElse(false.B)) {
      conflict_flags(Mux(hit, hit_idx, enq_idx)) := true.B
      conflict_way_mask := io.conflict_way_mask.get
      conflict_data := io.conflict_write_data.get
    }
    when (io.conflict_clean.getOrElse(false.B)) {
      conflict_flags(conflict_idx) := false.B
    }
    // for update the cached data
    io.has_conflict.get := conflict_flags.reduce(_||_)
    io.update_idx.get := idx_tag_cam.io.rdata.get.idx
    io.update_way_mask.get := conflict_way_mask
    io.update_data.foreach(_ := conflict_data)
  } else None

  XSPerfAccumulate("wrbypass_hit",  io.wen &&  hit)
  XSPerfAccumulate("wrbypass_miss", io.wen && !hit)

  XSDebug(io.wen && hit,  p"wrbypass hit entry #${hit_idx}, idx ${io.write_idx}" +
    p"tag ${io.write_tag.getOrElse(0.U)}data ${io.write_data}\n")
  XSDebug(io.wen && !hit, p"wrbypass enq entry #${enq_idx}, idx ${io.write_idx}" +
    p"tag ${io.write_tag.getOrElse(0.U)}data ${io.write_data}\n")
}

class WrBypass_v2[T <: Data](gen: T, val nEntries: Int, val idxWidth: Int, val nWays: Int = 2)(implicit p: Parameters) extends XSModule {

  require(nEntries >= 0)
  require(idxWidth >= 0)
  require(nWays >= 2)

  val io = IO(new Bundle {
    val ren = Input(Bool())
    val read_idx = Input(UInt(idxWidth.W))
    val read_hit_data = Vec(nWays, Valid(gen))

    val wen = Input(Bool())
    val write_idx = Input(UInt(idxWidth.W))
    val write_data = Input(Vec(nWays, gen))
    val write_way_mask = Input(Vec(nWays, Bool()))
    val write_hit = Output(Bool())
    val write_hit_data = Vec(nWays, Valid(gen))

    val sync_en = Output(Bool())
    val sync_idx = Output(UInt(idxWidth.W))
    val sync_data = Output(Vec(nWays, gen))
    val sync_way_mask = Output(Vec(nWays, Bool()))
  })

  val validVec = RegInit(0.U.asTypeOf(Vec(nEntries, Vec(nWays, Bool()))))
  val ageVec = RegInit(0.U.asTypeOf(Vec(nEntries, UInt(log2Ceil(nEntries).W))))
  val idxVec = RegInit(0.U.asTypeOf(Vec(nEntries, UInt(idxWidth.W))))
  val dataVec = Reg(Vec(nEntries, Vec(nWays, gen)))
  val writeMaskVec = Reg(Vec(nEntries, Vec(nWays, Bool())))

  val validVecNx = WireDefault(validVec)
  val ageVecNx = WireDefault(ageVec)
  val idxVecNx = WireDefault(idxVec)
  val dataVecNx = WireDefault(dataVec)
  val writeMaskVecNx = WireDefault(writeMaskVec)

  val read_hit_oh = validVec.zip(ageVec).zip(idxVec).map{ case((valid, age), idx) => valid.reduce(_||_) && age > 0.U && idx === io.read_idx }
  val read_hit_idx = OHToUInt(read_hit_oh)
  val read_hit = read_hit_oh.reduce(_||_)

  val write_hit_oh = validVec.zip(idxVec).map{ case(valid, idx) => valid.reduce(_||_) && idx === io.write_idx }
  val write_hit_idx = OHToUInt(write_hit_oh)
  val write_hit = write_hit_oh.reduce(_||_)
  val data_write_idx = Wire(UInt(log2Ceil(nEntries).W))

  // Replacer
  // Because data_mem can only write to one index
  // Implementing a per-way replacer is meaningless
  // So here use one replacer for all ways
  val replacer = ReplacementPolicy.fromString("plru", nEntries) // numEntries in total
  val enq_idx = replacer.way
  data_write_idx := Mux(write_hit, write_hit_idx, enq_idx)

  val replacer_touch_ways = Wire(Vec(1, Valid(UInt(log2Ceil(nEntries).W)))) // One index at a time
  replacer_touch_ways(0).valid := io.wen
  replacer_touch_ways(0).bits := data_write_idx
  replacer.access(replacer_touch_ways)

  when(io.wen){
    idxVecNx(data_write_idx) := Mux(write_hit, idxVec(data_write_idx), io.write_idx)
    for(i <- 0 until nWays){
      writeMaskVecNx(data_write_idx)(i) := io.write_way_mask(i) || writeMaskVec(data_write_idx)(i)
      when(io.write_way_mask(i)){
        dataVecNx(data_write_idx)(i) := io.write_data(i)
        validVecNx(data_write_idx)(i) := true.B
      }
    }
  }

  // sync logic
  val sync_age = ParallelMax(ageVec)
  val sync_oh = ageVec.map(_ === sync_age)
  val sync_idx = Mux1H(sync_oh, idxVec)
  val sync_data = Mux1H(sync_oh, dataVec)
  val sync_mask = Mux1H(sync_oh, writeMaskVec)

  val age_clr = sync_oh.map(_ && io.sync_en)
  val age_set = UIntToOH(data_write_idx).asBools.map(_ && io.wen && (io.ren || sync_age > 0.U))
  val age_inc = ageVec.map( age=> age =/= 0.U && age_set.reduce(_||_) )

  // age -> age + 1 if wen and ren and age_inc
  // age -> 0       if sync_en
  // age -> 1       if sync_en and wen and widx === syncidx
  // age -> age     other
  for(i <- 0 until nEntries){
    when(age_clr(i)){
      ageVecNx(i) := 0.U
      writeMaskVecNx(i) := 0.U.asTypeOf(writeMaskVecNx(i))
    }.elsewhen(age_set(i)){
      ageVecNx(i) := 1.U
    }.elsewhen(age_inc(i)){
      ageVecNx(i) := Mux(ageVec(i).andR, ageVec(i), ageVec(i) + 1.U)
    }.otherwise{
      ageVecNx(i) := ageVec(i)
    }
  }

  validVec := validVecNx
  ageVec := ageVecNx
  idxVec := idxVecNx
  dataVec := dataVecNx
  writeMaskVec := writeMaskVecNx

  io.read_hit_data.zipWithIndex.foreach{ case (read, i) =>
    read.valid := read_hit && writeMaskVec(read_hit_idx)(i)
    read.bits := dataVec(read_hit_idx)(i)
  }

  io.write_hit := write_hit
  io.write_hit_data.zipWithIndex.foreach{ case (write, i) =>
    write.valid := write_hit// && writeMaskVec(write_hit_idx)(i)
    write.bits := dataVec(write_hit_idx)(i)
  }

  io.sync_en := !io.ren && (io.wen || sync_age > 0.U) && io.sync_way_mask.reduce(_||_)
  io.sync_idx := Mux(sync_age > 0.U, sync_idx, io.write_idx)
  io.sync_data := Mux(sync_age > 0.U, sync_data, io.write_data)
  io.sync_way_mask := Mux(sync_age > 0.U, sync_mask, io.write_way_mask)

  XSError(PopCount(read_hit_oh) > 1.U, "multiple read hits")
  XSError(PopCount(write_hit_oh) > 1.U, "multiple write hits")
  XSError(io.ren && io.sync_en, "read and sync conflict")
  XSPerfAccumulate("wrbypass_sync_block",  io.ren && (io.wen || sync_age > 0.U))
  XSPerfAccumulate("wrbypass_age",  sync_age)

  XSPerfAccumulate("wrbypass_hit",  io.wen &&  write_hit)
  XSPerfAccumulate("wrbypass_miss", io.wen && !write_hit)
  XSDebug(io.wen && write_hit,  p"wrbypass hit entry #${write_hit_idx}, idx ${io.write_idx}\n")
  XSDebug(io.wen && !write_hit, p"wrbypass enq entry #${enq_idx}, idx ${io.write_idx}\n")

}
