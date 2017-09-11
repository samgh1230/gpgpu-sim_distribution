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


//Constructor
Scoreboard::Scoreboard( unsigned sid, unsigned n_warps )
: longopregs()
{
	m_sid = sid;
	//Initialize size of table
	reg_table.resize(n_warps);
	longopregs.resize(n_warps);
///add by gh
    pending_accesses.resize(n_warps);
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
    for(index=0;index<MAX_WARP_SIZE;index++){
        if(active_mask[index]){
            unsigned tid = inst->get_thread_id(index);
            unsigned wid = tid/MAX_WARP_SIZE;
            unsigned lane = tid%MAX_WARP_SIZE;
            if(reg_table[wid][regnum].test(lane)){
                printf("Error: trying to reserve an already reserved register (sid=%d, wid=%d, regnum=%d).", m_sid, wid, regnum);
                abort();
            }
            SHADER_DPRINTF( SCOREBOARD,
                "Reserved Register - warp:%d, reg: %d\n", wid, regnum );
            reg_table[wid][regnum].set(lane);
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
                printf("Error: trying to reserve an already reserved register (sid=%d, wid=%d, regnum=%d).", m_sid, wid, regnum);
                abort();
            }
            SHADER_DPRINTF( SCOREBOARD,
                "Reserved Register - warp:%d, reg: %d\n", wid, regnum );
            longopregs[wid][regnum].set(lane);
        }
    }
}
// Unmark register as write-pending
void Scoreboard::releaseRegister(unsigned regnum, warp_inst_t* inst)
{
    active_mask_t active_mask = inst->get_active_mask();
    int i;
    for(i=0;i<MAX_WARP_SIZE;i++){
        if(active_mask[i]){
            unsigned tid = inst->get_thread_id(i);
            unsigned wid = tid/MAX_WARP_SIZE;
            unsigned lane = tid%MAX_WARP_SIZE;
            reg_table[wid][regnum].reset(lane);
            longopregs[wid][regnum].reset(lane);
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

	//return longopregs[warp_id].find(regnum) != longopregs[warp_id].end();
}

void Scoreboard::reserveRegisters(class warp_inst_t* inst)
{
    for( unsigned r=0; r < 4; r++) {
        if(inst->out[r] > 0) {
            reserveRegister(inst->out[r], inst);
            SHADER_DPRINTF( SCOREBOARD,
                            "Reserved register - warp:%d, reg: %d\n",
                            inst->warp_id(),
                            inst->out[r] );
        }
    }

    //Keep track of long operations
    if (inst->is_load() &&
    		(	inst->space.get_type() == global_space ||
    			inst->space.get_type() == local_space ||
                inst->space.get_type() == param_space_kernel ||
                inst->space.get_type() == param_space_local ||
                inst->space.get_type() == param_space_unclassified ||
    			inst->space.get_type() == tex_space)){
    	for ( unsigned r=0; r<4; r++) {
    		if(inst->out[r] > 0) {
                SHADER_DPRINTF( SCOREBOARD,
                                "New longopreg marked - warp:%d, reg: %d\n",
                                inst->warp_id(),
                                inst->out[r] );
                //longopregs[inst->warp_id()].insert(inst->out[r]);
                reserveLopRegister(inst->out[r],inst);
                //added by gh
                //pending_accesses[inst->warp_id()][inst].insert(inst->out[r]);
            }
    	}
    }
}

// Release registers for an instruction
void Scoreboard::releaseRegisters(class warp_inst_t *inst)
{
    for( unsigned r=0; r < 4; r++) {
        if(inst->out[r] > 0) {
            SHADER_DPRINTF( SCOREBOARD,
                            "Register Released - warp:%d, reg: %d\n",
                            inst->warp_id(),
                            inst->out[r] );
            releaseRegister(inst->out[r],inst);
            //longopregs[inst->warp_id()].erase(inst->out[r]);
        }
    }
    //added by gh
    pending_accesses[inst->warp_id()].erase(inst);
}

/**
 * Checks to see if registers used by an instruction are reserved in the scoreboard
 *
 * @return
 * true if WAW or RAW hazard (no WAR since in-order issue)
 **/
bool Scoreboard::checkCollision( class warp_inst_t *inst )
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
    active_mask_t active_mask = inst->get_active_mask();
    for ( it2=inst_regs.begin() ; it2 != inst_regs.end(); it2++ ){
        unsigned i;
        for(i=0;i<MAX_WARP_SIZE;i++){
            if(active_mask.test(i)){
                unsigned tid = inst->get_thread_id(i);
                unsigned wid = tid/MAX_WARP_SIZE;
                unsigned lane = tid%MAX_WARP_SIZE;
                if(reg_table[wid][*it2].test(lane))
                    return true;
            }
        }
    }
    return false;
		//if(reg_table[wid].find(*it2) != reg_table[wid].end()) {
			//return true;
		//}
	//return false;
}
//check load dependence, added by gh
bool Scoreboard::checkCollisionLD(class warp_inst_t *inst) {
    if(checkCollision(inst))
    {
        std::set<int> inst_regs;

    	if(inst->in[0] > 0) inst_regs.insert(inst->in[0]);
	    if(inst->in[1] > 0) inst_regs.insert(inst->in[1]);
    	if(inst->in[2] > 0) inst_regs.insert(inst->in[2]);
	    if(inst->in[3] > 0) inst_regs.insert(inst->in[3]);


    	// Check for collision, get the intersection of reserved registers and instruction registers
	    std::set<int>::const_iterator it2;
        active_mask_t active_mask = inst->get_active_mask();
        for ( it2=inst_regs.begin() ; it2 != inst_regs.end(); it2++ ){
            unsigned i;
            for(i=0;i<MAX_WARP_SIZE;i++){
                if(active_mask.test(i)){
                     unsigned tid = inst->get_thread_id(i);
                    unsigned wid = tid/MAX_WARP_SIZE;
                    unsigned lane = tid%MAX_WARP_SIZE;
                    if(longopregs[wid][*it2].test(lane))
                        return true;
                }
            }
        }
	    return false;
    }
    else false;
}

//get depend inst, added by gh
warp_inst_t* Scoreboard::getDepInst(unsigned wid, const class warp_inst_t *inst)
{
    std::set<int> inst_regs;
    if(inst->in[0] > 0) inst_regs.insert(inst->in[0]);
    if(inst->in[1] > 0) inst_regs.insert(inst->in[1]);
    if(inst->in[2] > 0) inst_regs.insert(inst->in[2]);
    if(inst->in[3] > 0) inst_regs.insert(inst->in[3]);

    std::set<int>::iterator it;
    std::map<warp_inst_t*, std::set<unsigned> >::iterator it_inst = pending_accesses[wid].begin();
    for(;it_inst!=pending_accesses[wid].end();it_inst++){
        for(it=inst_regs.begin();it!=inst_regs.end();it++){
            if(it_inst->second.find(*it) != it_inst->second.end()){
                return it_inst->first;
            }
        }
    }
    return NULL;
}

bool Scoreboard::pendingWrites(unsigned wid) const
{
	return !reg_table[wid].empty();
}
