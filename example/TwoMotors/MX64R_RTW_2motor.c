/*
 * Comunicate Two MX64R Dynamixel motor in 3 cycle  with a level 2 S-function.
 * 1.Reading from first motor 2.Reading from second motor 3. Sync Write on two motors. 
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

#define S_FUNCTION_NAME  MX64R_RTW_2motor
#define S_FUNCTION_LEVEL 2
#define param_1 *mxGetPr(ssGetSFcnParam(S,0))
#define param_2 *mxGetPr(ssGetSFcnParam(S,1))

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include "stdlib.h"
#include "stdio.h"

//Global variable
  int_T id1,id2,cycle;
  uint8_T  data[16] = { 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0,0,0};
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
      
    ssSetNumSFcnParams(S, 2);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S,"Parameter assignment failed! \nplease enter the parameter as the following: first dynamixel ID,second Dynamixel ID");
        return;
    }
    
    cycle=0;
    
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 3)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortWidth(S, 2, 12);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    if (!ssSetNumOutputPorts(S, 7)) return;
    ssSetOutputPortDataType(S, 6, SS_UINT8);
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1 , 1);
    ssSetOutputPortWidth(S, 2 , 1);
    ssSetOutputPortWidth(S, 3 , 1);
    ssSetOutputPortWidth(S, 4 , 1);
    ssSetOutputPortWidth(S, 5 , 1);
    ssSetOutputPortWidth(S, 6 , 16);

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
    real_T a1,a2;
    time_T t,tf;
    int_T checksum ,high1,low1,high2,low2,i,id1,id2;
    InputRealPtrsType uPtrs1,uPtrs2,uPtrs3;
    real_T *pos1,*veloc1,*load1,*pos2,*veloc2,*load2,TRQIN1,TRQIN2,cl1,cl2,cs1,cs2;
    uint8_T *serialout= (uint8_T *)ssGetOutputPortSignal(S,6);
    
    uPtrs1=ssGetInputPortRealSignalPtrs(S,0);
    uPtrs2=ssGetInputPortRealSignalPtrs(S,1);
    uPtrs3=ssGetInputPortRealSignalPtrs(S,2);
    
    pos1=ssGetOutputPortRealSignal(S,0);
    veloc1=ssGetOutputPortRealSignal(S,1);
    load1=ssGetOutputPortRealSignal(S,2);
    pos2=ssGetOutputPortRealSignal(S,3);
    veloc2=ssGetOutputPortRealSignal(S,4);
    load2=ssGetOutputPortRealSignal(S,5);
   
  
    TRQIN1=*uPtrs1[0];
    TRQIN2=*uPtrs2[0];
    
    id1=param_1;
    id2=param_2;
    
      //read from serial port motor1
     if(cycle==0 )
     {
          
         data[2]=id1;
         data[3]=0x04;
         data[4]=0x02;
         data[5]=0x24;
         data[6]=0x06;
         checksum=~(id1+4+2+0x24+6);
         data[7]=checksum;
        
         
        for(i=0;i<16;i++)
        serialout[i]=data[i];
      
  
     }
    
       //read from serial port motor2
             if(cycle==2 )
     {
          
         data[2]=id2;
         data[3]=0x04;
         data[4]=0x02;
         data[5]=0x24;
         data[6]=0x06;
         checksum=~(id2+4+2+0x24+6);
         data[7]=checksum;
        
         
        for(i=0;i<16;i++)
        serialout[i]=data[i];
        
  
      
     }
             
             
             
       for (i=0; i<12; i++) 
       output[i] = (uint8_T)*uPtrs3[i];
         
       
       if(output[2]==id1 ){
     *pos1=(uint16_T)((output[5])+(uint16_T)(output[6])*256); 
      cs1=(uint16_T)((output[7])+(uint16_T)(output[8])*256);
      cl1=(uint16_T)((output[9])+(uint16_T)(output[10])*256);
      
      if((cs1<=1023) & (cs1>=0))
         *veloc1=cs1;
      else if ((cs1>=1024) & (cs1<=2047))
          *veloc1=(-cs1+1024);
      
      if((cl1<=1023) & (cl1>=0))
         *load1=cl1;
      else if ((cl1>=1024) & (cl1<=2047))
          *load1=(-cl1+1024);}
    
      
       if(output[2]==id2 ){
      *pos2=(uint16_T)((output[5])+(uint16_T)(output[6])*256); 
      cs2=(uint16_T)((output[7])+(uint16_T)(output[8])*256);
      cl2=(uint16_T)((output[9])+(uint16_T)(output[10])*256);
      
      if((cs2<=1023) & (cs2>=0))
         *veloc2=cs2;
      else if ((cs2>=1024) & (cs2<=2047))
          *veloc2=(-cs2+1024);
      
      if((cl2<=1023) & (cl2>=0))
         *load2=cl2;
      else if ((cl1>=1024) & (cl1<=2047))
          *load2=(-cl2+1024);}

    
 
    
 //write to serial port
      if (cycle==1)
      {
    if ((TRQIN1>=0) & (TRQIN1<=6))
        a1= TRQIN1*1023/6;
        else if((TRQIN1<0) & (TRQIN1>=-6))
        a1=-TRQIN1*1023/6+1024;
        else 
         a1=0;
    
     if ((TRQIN2>=0) & (TRQIN2<=6))
        a2= TRQIN2*1023/6;
        else if((TRQIN2<0) & (TRQIN2>=-6))
        a2=-TRQIN2*1023/6+1024;
        else 
         a2=0;
    
    high1=(int_T)(a1/256);
    low1=(int_T)(a1-high1*256);
    high2=(int_T)(a2/256);
    low2=(int_T)(a2-high2*256);
    
    data[2]=0xfe;
    data[3]=0x0c;
    data[4]=0x83;
    data[5]=0x46;
    data[6]=0x03;
    data[7]=id1;
    data[8]=0x01;
    data[9]=low1;
    data[10]=high1;
    data[11]=id2;
    data[12]=0x01;
    data[13]=low2;
    data[14]=high2;
    checksum=~(0xfe +0x0c+ 0x83+ 0x46+ 0x03+id1+ 0x01+ low1+ high1+ id2+ 0x01+ low2+ high2);
    data[15]=checksum;
    
      for(i=0;i<16;i++)
      serialout[i]=data[i];
     }  
      
    
    // Termination Assignment
       t= ssGetT(S);
       tf= ssGetTFinal(S);
       if(t==tf){
    data[2]=0xfe;
    data[3]=0x0c;
    data[4]=0x83;
    data[5]=0x46;
    data[6]=0x03;
    data[7]=id1;
    data[8]=0x01;
    data[9]=0;
    data[10]=0;
    data[11]=id2;
    data[12]=0x01;
    data[13]=0;
    data[14]=0;
    checksum=~(0xfe +0x0c+ 0x83+ 0x46+ 0x03+id1+ 0x01+ id2+ 0x01);
    data[15]=checksum;
    
      for(i=0;i<16;i++)
      serialout[i]=data[i];   
       }
    
    
  cycle++;
  if (cycle==3)
     cycle=0;
   
     
}

/* Function: mdlTerminate =====================================================
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
