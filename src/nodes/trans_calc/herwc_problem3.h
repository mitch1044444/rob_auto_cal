/* 
   This file was automatically generated on May 26, 2015 23:44:10
   by GpoSolver, (c) 2013--2014, Jan Heller, hellej1@cmp.felk.cvut.cz,
   Czech Technical University, Prague, Czech Republic 

   This solver takes:
      24 residual parameters: [ra1, ra4, ra7, ra2, ra5, ra8, ra3, ra6, ra9, ta1, ta2, ta3, rb1, rb4, rb7, rb2, rb5, rb8, rb3, rb6, rb9, tb1, tb2, tb3]
       2 problem parameters : [n, m]
*/

#ifndef HERWC_PROBLEM3_H
#define HERWC_PROBLEM3_H

#include <rob-auto-cal/gpoproblem.h>

namespace GpoSolver {

class HerwcProblem : public GpoProblem {
  private:
    static const int num_blocks;
    static const int num_cons;
    static const int num_pcons;
    static const int num_pvars;
    static const int num_rpars;
    static const int num_ppars;
    static const int problem_size;
    static const int relax_order;
    static const int max_degree;
    static const int rank_shift;
    static const int block_sizes[];
 
    static const int num_obj_data;
    static const int num_obj_vars;
    static const objData obj_data[];
    static const objVar obj_vars[];

    static const int num_con_data;
    static const int num_con_vars;
    static const int num_con_blocks;
    static const conData con_data[];
    static const conVar con_vars[];
    static const conBlock con_blocks[];

  public:
  
    int getNumBlocks(void) const {
      return num_blocks;
    }
 
    int getNumConstraints(void) const {
      return num_cons;
    }
 
    int getNumProblemVariables(void) const {
      return num_pvars;
    }
 
    int getNumPolyConstraints(void) const {
      return num_pcons;
    }
 
    int getNumObjectiveParams(void) const {
      return num_rpars;
    }
 
    int getNumConstraintsParams(void) const {
      return num_ppars;
    }
 
    int getProblemSize(void) const {
      return problem_size;
    }
 
    int getRelaxationOrder(void) const {
      return relax_order;
    }
 
    int getMaxDegree(void) const {
      return max_degree;
    }
 
    int getRankShift(void) const {
      return rank_shift;
    }
 
    const int * getBlockSizes(void) const {
      return block_sizes;
    }
 
    int getBlockSize(int i) const {
      return block_sizes[i];
    }
 
    int getNumSdpObjectiveData(void) const {
      return num_obj_data;
    }
 
    int getNumSdpObjectiveVariables(void) const {
      return num_obj_vars;
    }
 
    const objData * getSdpObjectiveData(void) const {
      return obj_data;
    }
 
    const objVar * getSdpObjectiveVariables(void) const {
      return obj_vars;
    }
 
    const conData * getSdpConstraintsData(void) const {
      return con_data;
    }
 
    const conVar * getSdpConstraintsVariables(void) const {
      return con_vars;
    }
    int getNumSdpConstraintsData(void) const {
      return num_con_data;
    }
 
    int getNumSdpConstraintsVariables(void) const {
      return num_con_vars;
    }
 
    void updateAuxiliaries(const double *, double *) const;
    double evalPolyObjective(const double *, const double *, const double *) const;
    void evalPolyConstraints(const double *, const double *, double *) const;
    void evalSdpObjectiveVariables(const double *, const double *, double **) const;
    void evalSdpConstraintsVariables(const double *, const double *, double **) const;

}; // HerwcProblem

} // GPOSolver namespace 

#endif // HERWC_PROBLEM3_H

