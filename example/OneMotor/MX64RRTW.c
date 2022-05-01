/*
 * Comunicate One MX64R Dynamixel motor  with a level 2 S-function.
 * By Pedram Agand
 * 
 *
 * Copyright 2014 Aras Lab, K.N. Tossi university of technology.
 * 
 * For more Information Contact: pedram_akand@yahoo.com
 */


/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  MX64RRTW
#define S_FUNCTION_LEVEL 2
#define param_1 *mxGetPr(ssGetSFcnParam(S,0))


/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include "stdlib.h"
#include "stdio.h"

//Global variable
  int_T id,cycle;
  uint8_T  data[10] = { 0xff, 0xff, 0, 0, 0,0, 0, 0, 0,0};
  uint8_T  output[12] = {0, 0,0,0,0,0,0,0,0,0,0,0}; 

/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* See sfuntmpl_doc.c for more details on the macros below */
      
    ssSetNumSFcnParams(S,1);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S,"Parameter assignment failed! \nplease enter the ID of dynamixel as the parameter ");
        return;
    }
    id=param_1;
    data[2]=id;
    cycle=0;
    

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortWidth(S, 1, 12);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    

    if (!ssSetNumOutputPorts(S, 4)) return;
    ssSetOutputPortDataType(S, 3, SS_UINT8);
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1 , 1);
    ssSetOutputPortWidth(S, 2 , 1);
    ssSetOutputPortWidth(S, 3 , 10);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);

}



/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);

}
 

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T a;
    time_T t,tf;
    int_T checksum ,high,low,i;
    InputRealPtrsType uPtrs1,uPtrs2;
    real_T *pos,*veloc,*load,TRQIN,cl,cs;
    uint8_T *serialout= (uint8_T *)ssGetOutputPortSignal(S,3);
    
    uPtrs1=ssGetInputPortRealSignalPtrs(S,0);
    uPtrs2=ssGetInputPortRealSignalPtrs(S,1);
    
    pos=ssGetOutputPortRealSignal(S,0);
    veloc=ssGetOutputPortRealSignal(S,1);
    load=ssGetOutputPortRealSignal(S,2);
    TRQIN=*uPtrs1[0];
  
      //read from serial port
     if(cycle==0  )
     {
         data[3]=0x04;
         data[4]=0x02;
         data[5]=0x24;
         data[6]=0x06;
         checksum=~(id+4+2+0x24+6);
         data[7]=checksum;
        
         
        for(i=0;i<10;i++)
        serialout[i]=data[i];
        
       for (i=0; i<12; i++) 
       output[i] = (uint8_T)*uPtrs2[i];
       
      *pos=(uint16_T)((output[5])+(output[6])*256); 
      cs=(uint16_T)((output[7])+(output[8])*256);
      cl=(uint16_T)((output[9])+(output[10])*256);
      
      if((cs<=1023) & (cs>=0))
         *veloc=cs;
      else if ((cs>=1024) & (cs<=2047))
          *veloc=(-cs+1024);
      
      if((cl<=1023) & (cl>=0))
         *load=cl;
      else if ((cl>=1024) & (cl<=2047))
          *load=(-cl+1024);
    
      
     }
    
 
    
 //write to serial port
     if (cycle==1)
      {
    if ((TRQIN>=0) & (TRQIN<=6))
        a= TRQIN*1023/6;
        else if((TRQIN<0) & (TRQIN>=-6))
        a=-TRQIN*1023/6+1024;
        else 
         a=0;
    high=(int_T)(a/256);
    low=(int_T)(a-high*256);
    data[3]=0x06;
    data[4]=0x03;
    data[5]=0x46;
    data[6]=0x01;
    data[7]=low;
    data[8]=high;
    checksum=~(id+6+3+1+high+low+0x46);
    data[9]=checksum;
    
    
      for(i=0;i<10;i++)
      serialout[i]=data[i];
     }  
      
       t= ssGetT(S);
       tf= ssGetTFinal(S);
       if(t==tf){
    data[3]=0x06;
    data[4]=0x03;
    data[5]=0x46;
    data[6]=0x01;
    data[7]=0;
    data[8]=0;
    checksum=~(id+6+3+1+0x46);
    data[9]=checksum;
    
      for(i=0;i<10;i++)
      serialout[i]=data[i];   
       }
       
       
    if(cycle==0)
    cycle=1;
     else
     cycle=0;

}

/* Function: mcdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
}


/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
