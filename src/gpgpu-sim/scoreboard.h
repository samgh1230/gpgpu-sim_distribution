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

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <set>
#include "assert.h"

#ifndef SCOREBOARD_H_
#define SCOREBOARD_H_

#include "../abstract_hardware_model.h"


class Scoreboard {
public:
    Scoreboard( unsigned sid, unsigned n_warps );

    void reserveRegisters( warp_inst_t *inst);
    void releaseRegisters(warp_inst_t *inst);
    void releaseRegister(unsigned regnum,warp_inst_t* inst);

    bool checkCollision(warp_inst_t *inst) ;
    //check load dependence, added by gh
    bool checkCollisionLD(class warp_inst_t *inst) ;
    warp_inst_t* getDepInst(unsigned wid, const class warp_inst_t *inst);

    bool pendingWrites(unsigned wid) const;
    //void printContents() const;
    const bool islongop(unsigned regnum,warp_inst_t* inst);
private:
    void reserveRegister(unsigned regnum, class warp_inst_t* inst);
    void reserveLopRegister(unsigned regnum, warp_inst_t* inst);
    int get_sid() const { return m_sid; }

    unsigned m_sid;

    // keeps track of pending writes to registers
    // indexed by warp id, reg_id => pending write count
    //std::vector< std::set<unsigned> > reg_table;
    //Register that depend on a long operation (global, local or tex memory)
    //std::vector< std::set<unsigned> > longopregs;

    std::vector< std::map< warp_inst_t*, std::set<unsigned> > > pending_accesses;

    std::vector< std::vector< std::bitset<MAX_WARP_SIZE> > >  reg_table;//wid:reg_id:thread_active
    std::vector< std::vector< std::bitset<MAX_WARP_SIZE> > >  longopregs;//wid:reg_id:thread_active
};


#endif /* SCOREBOARD_H_ */
