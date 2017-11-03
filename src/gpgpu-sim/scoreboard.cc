// Copyright (c) 2009-2011, Tor M. Aamodt, Inderpreet Singh
// The University of British Columbia
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice, this
// list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.
// Neither the name of The University of British Columbia nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "scoreboard.h"
#include "shader.h"
#include "../cuda-sim/ptx_sim.h"
#include "shader_trace.h"
#include "../cuda-sim/ptx-stats.h"
#include "../cuda-sim/cuda-sim.h"
#define NUM_REG 32768
//Constructor
Scoreboard::Scoreboard( unsigned sid, unsigned n_warps, shader_core_ctx* shader )
: longopregs()
{
	m_sid = sid;
	//Initialize size of table
	reg_table.resize(n_warps);
	longopregs.resize(n_warps);
    pending_write_per_thread.resize(shader->get_config()->n_thread_per_shader,0);
    unsigned i;
    for(i=0;i<n_warps;i++){
        reg_table[i].resize(NUM_REG);
        longopregs[i].resize(NUM_REG);
        unsigned j;
        for(j=0;j<NUM_REG;j++){
            reg_table[i][j].reset();
            longopregs[i][j].reset();
        }
    }
    m_shader = shader;
}

// Print scoreboard contents
//void Scoreboard::printContents() const
//{
	//printf("scoreboard contents (sid=%d): \n", m_sid);
	//for(unsigned i=0; i<reg_table.size(); i++) {
		//if(reg_table[i].size() == 0 ) continue;
		//printf("  wid = %2d: ", i);
		//std::set<unsigned>::const_iterator it;
		//for( it=reg_table[i].begin() ; it != reg_table[i].end(); it++ )
			//printf("%u ", *it);
		//printf("\n");
	//}
//}

void Scoreboard::reserveRegister(unsigned regnum, class warp_inst_t* inst)
{
    active_mask_t active_mask = inst->get_active_mask();
    int index;
    SHADER_DPRINTF( SCOREBOARD, "Reserved Register - sid:%d, reg:%d\n ",m_sid, regnum );
    for(index=0;index<MAX_WARP_SIZE;index++){
        if(active_mask[index]){
            unsigned tid = inst->get_thread_id(index);
            unsigned wid = tid/MAX_WARP_SIZE;
            unsigned lane = tid%MAX_WARP_SIZE;
            reg_table[wid][regnum].set(lane);
            pending_write_per_thread[tid]++;
            SHADER_DPRINTF( SCOREBOARD, "warp:%d, tid:%d, active_count:%d.(%s)\n",  wid, tid,active_mask.count(),ptx_get_insn_str(inst->pc).c_str());
        }
    }
}

void Scoreboard::reserveLopRegister(unsigned regnum, class warp_inst_t* inst)
{
	active_mask_t active_mask = inst->get_active_mask();
    int index;
    for(index=0;index<MAX_WARP_SIZE;index++){
        if(active_mask[index]){
            unsigned tid = inst->get_thread_id(index);
            unsigned wid = tid/MAX_WARP_SIZE;
            unsigned lane = tid%MAX_WARP_SIZE;
            if(longopregs[wid][regnum].test(lane)){
                printf("Error: trying to reserve an already reserved long op register (sid=%d, wid=%d,tid:%d, regnum=%d, %s\n).", m_sid, wid,tid, regnum, ptx_get_insn_str(inst->pc).c_str());
                abort();
            }
            longopregs[wid][regnum].set(lane);
        }
    }
}
// Unmark register as write-pending
void Scoreboard::releaseRegister(unsigned regnum, warp_inst_t* inst)
{
    SHADER_DPRINTF( SCOREBOARD, "Release Register - sid:%d, reg:%d\n", m_sid, regnum );
    active_mask_t active_mask = inst->get_active_mask();
    /*if(regnum==18&&m_sid==6&&inst->warp_id()==0)*/
        /*printf("active_mask:%x\n",active_mask.to_ulong());*/
    int i;
    for(i=0;i<MAX_WARP_SIZE;i++){
        if(active_mask[i]){
            unsigned tid = inst->get_thread_id(i);
            unsigned wid = tid/MAX_WARP_SIZE;
            unsigned lane = tid%MAX_WARP_SIZE;
            if(reg_table[wid][regnum].test(lane)){
                assert(pending_write_per_thread[tid]>0);
                pending_write_per_thread[tid]--;
            }
            reg_table[wid][regnum].reset(lane);
            longopregs[wid][regnum].reset(lane);

            SHADER_DPRINTF( SCOREBOARD, "warp:%d, tid:%d, active_count:%d.(%s)\n", wid, tid, active_mask.count(),ptx_get_insn_str( inst->pc).c_str());
        }
    }
}

void Scoreboard::releaseRegister(unsigned regnum, warp_inst_t* inst,mem_fetch* mf){
    active_mask_t active_mask=mf->get_access_warp_mask();

    for(unsigned i=0;i<MAX_WARP_SIZE;i++){
        if(active_mask[i]){
            unsigned tid = inst->get_thread_id(i);
            unsigned wid = tid/MAX_WARP_SIZE;
            unsigned lane = tid%MAX_WARP_SIZE;
            if(reg_table[wid][regnum].test(lane)){
                assert(pending_write_per_thread[tid]>0);
                pending_write_per_thread[tid]--;
            }
            reg_table[wid][regnum].reset(lane);
            longopregs[wid][regnum].reset(lane);
            SHADER_DPRINTF( SCOREBOARD, "warp:%d, tid:%d, active_count:%d.(%s)\n", wid, tid, active_mask.count(),ptx_get_insn_str( inst->pc).c_str());
        }
    }
}

const bool Scoreboard::islongop (unsigned regnum, warp_inst_t* inst) {
    active_mask_t active_mask = inst->get_active_mask();
    int i;
    for(i=0;i<MAX_WARP_SIZE;i++){
        if(active_mask[i]){
            unsigned tid = inst->get_thread_id(i);
            unsigned wid = tid/MAX_WARP_SIZE;
            unsigned lane = tid%MAX_WARP_SIZE;
            if(longopregs[wid][regnum].test(lane))
                return true;
        }
    }
    return false;

}

void Scoreboard::reserveRegisters(class warp_inst_t* inst)
{
    for( unsigned r=0; r < 4; r++) {
        if(inst->out[r] > 0) {
            reserveRegister(inst->out[r], inst);
        }
    }

    //Keep track of long operations
    if (inst->is_load() && inst->get_num_access()>1 &&
            (	inst->space.get_type() == global_space ||
                inst->space.get_type() == local_space )){
        for ( unsigned r=0; r<4; r++) {
            if(inst->out[r] > 0) {
                reserveLopRegister(inst->out[r],inst);
            }
        }
    }
    /*if(inst->is_load() && inst->get_num_access()==1 && 
        inst->get_active_mask().count()==1 &&
        inst->get_dwf_flag() &&
        ( inst->space.get_type()==global_space ||
          inst->space.get_type()==local_space)) {
        for(unsigned r=0;r<4;r++){
            if(inst->out[r]>0)
                reserveLopRegister(inst->out[r],inst);
        }
    }*/
}


// Release registers for an instruction
void Scoreboard::releaseRegisters(class warp_inst_t *inst)
{
    for( unsigned r=0; r < 4; r++) {
        if(inst->out[r] > 0) {
            releaseRegister(inst->out[r],inst);
        }
    }
}

void Scoreboard::releaseRegisters( mem_fetch* mf){
    warp_inst_t inst=mf->get_n_inst();
    for( unsigned r=0; r < 4; r++) {
        if(inst.out[r] > 0) {
            releaseRegister(inst.out[r],&inst,mf);
        }
    }
}

/**
 * Checks to see if registers used by an instruction are reserved in the scoreboard
 *
 * @return
 * true if WAW or RAW hazard (no WAR since in-order issue)
 **/
bool Scoreboard::checkCollision( class warp_inst_t *inst,unsigned warp_id)
{
	// Get list of all input and output registers
	std::set<int> inst_regs;

	if(inst->out[0] > 0) inst_regs.insert(inst->out[0]);
	if(inst->out[1] > 0) inst_regs.insert(inst->out[1]);
	if(inst->out[2] > 0) inst_regs.insert(inst->out[2]);
	if(inst->out[3] > 0) inst_regs.insert(inst->out[3]);
	if(inst->in[0] > 0) inst_regs.insert(inst->in[0]);
	if(inst->in[1] > 0) inst_regs.insert(inst->in[1]);
	if(inst->in[2] > 0) inst_regs.insert(inst->in[2]);
	if(inst->in[3] > 0) inst_regs.insert(inst->in[3]);
	if(inst->pred > 0) inst_regs.insert(inst->pred);
	if(inst->ar1 > 0) inst_regs.insert(inst->ar1);
	if(inst->ar2 > 0) inst_regs.insert(inst->ar2);

	// Check for collision, get the intersection of reserved registers and instruction registers
	std::set<int>::const_iterator it2;
    active_mask_t active_mask = m_shader->get_warp_by_index(warp_id).get_active_mask();
    for ( it2=inst_regs.begin() ; it2 != inst_regs.end(); it2++ ){
        unsigned i;
        for(i=0;i<MAX_WARP_SIZE;i++){
            if(active_mask.test(i)){
                unsigned tid = m_shader->get_warp_by_index(warp_id).get_thread_ids()[i];
                unsigned wid = tid/MAX_WARP_SIZE;
                unsigned lane = tid%MAX_WARP_SIZE;
                if(reg_table[wid][*it2].test(lane))
                    return true;
            }
        }
    }
    return false;
}
//check load dependence, added by gh
bool Scoreboard::checkCollisionLD(class warp_inst_t *inst, unsigned warp_id) {
        std::set<int> inst_regs;

    	if(inst->in[0] > 0) inst_regs.insert(inst->in[0]);
	    if(inst->in[1] > 0) inst_regs.insert(inst->in[1]);
    	if(inst->in[2] > 0) inst_regs.insert(inst->in[2]);
	    if(inst->in[3] > 0) inst_regs.insert(inst->in[3]);

        bool collision = false;
    	// Check for collision, get the intersection of reserved registers and instruction registers
	    std::set<int>::const_iterator it2,it3;
        std::set<unsigned> dep_regs;
        active_mask_t active_mask = m_shader->get_simt_stack(warp_id)->get_active_mask();
        unsigned i;
        for(i=0;i<MAX_WARP_SIZE;i++){
            collision=false;
            unsigned tid=m_shader->get_warp_by_index(warp_id).get_thread_ids()[i];
            for(it2=inst_regs.begin();it2!=inst_regs.end();it2++){
                if(active_mask.test(i)){
                    unsigned wid=tid/MAX_WARP_SIZE;
                    unsigned lane=tid%MAX_WARP_SIZE;
                    if(longopregs[wid][*it2].test(lane)){
                        collision=true;
                        break;
                    }
                }
            }
            if(collision){
                if(!m_shader->get_config()->gpgpu_dwf_enable)
                    return collision;
                //aggresive warp split,added by gh
                /*for(unsigned j=0;j<MAX_WARP_SIZE;j++){
                    std::deque<simt_stack_entry> stack_entry = m_shader->get_simt_stack(warp_id)->get_stack_entry();
                    unsigned tid=m_shader->get_warp_by_index(warp_id).get_thread_ids()[j];
                    for(it3=inst_regs.begin();it3!=inst_regs.end();it3++){
                        if(active_mask.test(j)){
                            unsigned wid=tid/MAX_WARP_SIZE;
                            unsigned lane=tid%MAX_WARP_SIZE;
                            if(longopregs[wid][*it3].test(lane)){
                                dep_regs.insert(*it3);
                            }
                        }
                    }
                    std::deque<simt_stack_entry>::iterator it;
                    for(it=stack_entry.begin();it!=stack_entry.end();){
                        if(!it->m_active_mask.test(j))
                            it = stack_entry.erase(it);
                        else{
                            it->m_active_mask.reset();
                            it->m_active_mask.set(j);
                            it++;
                        }
                    }
                    if(tid!=-1&&!stack_entry.empty()){
                        unsigned cta_id=m_shader->get_thread_ctx(tid).m_cta_id;
                        m_shader->get_dwf_unit()->add_thread2pool(inst,tid,dep_regs,stack_entry,cta_id);
                    }
                    dep_regs.clear();
                }*/
                //break;
            }
        }

        return collision;
}


bool Scoreboard::pendingWrites(unsigned wid) const
{
    active_mask_t mask=m_shader->get_warp_by_index(wid).get_active_mask();
    std::vector<unsigned> threads= m_shader->get_warp_by_index(wid).get_thread_ids();
    for(unsigned i=0;i<MAX_WARP_SIZE;i++){
        if(mask.test(i)){
            unsigned tid=threads[i];
            if(pending_write_per_thread[tid])
                return true;
            assert(pending_write_per_thread[tid]==0);
        }
    }
    return false;

}
