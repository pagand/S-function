/*
 * Comunicate One MX64R Dynamixel motor  with a level 2 S-function by self-Defining the Ports.
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

#define S_FUNCTION_NAME  MX64Rf
#define S_FUNCTION_LEVEL 2
#define param_1 *mxGetPr(ssGetSFcnParam(S,0))
#define param_2 *mxGetPr(ssGetSFcnParam(S,1))
#define param_3 *mxGetPr(ssGetSFcnParam(S,2))
#define param_4 *mxGetPr(ssGetSFcnParam(S,3))
#define PI 3.141592

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include "stdlib.h"
#include "stdio.h"
#include "windows.h"

//Global variable
  HANDLE serialHandle;
  DWORD dwBytesWritten;
  DWORD  dwBytesRead;
  DCB serialParams = { 0 };
  int_T isRead = false;
  int_T isWritten = false;
  int_T id;
  char  data[10] = { 0xff, 0xff, 0,0,0, 0,0, 0,0,0};
  char  output[8] = {0, 0,0,0,0,0,0,0};
   

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

    ssSetNumSFcnParams(S, 4);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        ssSetErrorStatus(S,"Parameter assignment failed! \nplease enter the parameter as the following port-num,boadrate,motor_id,control_methode\n exp:1,57600,1,0");
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);

    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1 , 1);

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

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
          int_T BoadRate,com;
          char *pcom[5];
    com=(int_T)param_1;
    BoadRate=(int_T)param_2;
    id=param_3;
    data[2]=id;
    
    switch (com){
    case 1:
        *pcom="COM1";
        break;
    case 2:
        *pcom="COM2";
        break;
    case 3:
        *pcom="COM3";
        break;
   case 4:
        *pcom="COM4";
        break;
  case 5:
        *pcom="COM5";
        break;
  case 6:
        *pcom="COM6";
        break;
  case 7:
        *pcom="COM7";
        break;
  case 8:
        *pcom="COM8";
        break;
  case 9:
        *pcom="COM9";
        break;
  case 10:
        *pcom="COM10";
        break;
  default:
        ssSetErrorStatus(S,"Invalid port name!");
        return; 
    }
            
    // Open serial port
serialHandle = CreateFile( *pcom , GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
// Do some basic settings
serialParams.fInX = false;
serialParams.fOutxCtsFlow = false;
serialParams.fOutxDsrFlow = false;
serialParams.fDsrSensitivity = false;
serialParams.fRtsControl = RTS_CONTROL_ENABLE;
serialParams.fDtrControl = DTR_CONTROL_DISABLE;


serialParams.DCBlength = sizeof(serialParams);
serialParams.BaudRate = BoadRate;
serialParams.ByteSize = 8;
serialParams.StopBits = ONESTOPBIT;
serialParams.Parity = NOPARITY;

if (!SetCommState(serialHandle,&serialParams)) 
{
ssSetErrorStatus(S,"Failed to Read the HANDLE!");
return; 
}  
   
  }
#endif /*  MDL_START */


int curr_load(SimStruct *S)
  {
        int_T  checksum ,cload;
    data[3]=0x04;
    data[4]=0x02;
    data[5]=0x28;
    data[6]=0x02;
    checksum=~(id+4+2+0x28+2);
    data[7]=checksum;
 
   // isWritten = WriteFile(serialHandle,&data,(DWORD) sizeof(data), &dwBytesWritten, NULL);

                    serialParams.fRtsControl = RTS_CONTROL_ENABLE; // manual RTS control and turns on RTS
                    SetCommState( serialHandle, &serialParams );     
        isWritten = WriteFile(serialHandle, (&data) ,10, &dwBytesWritten, NULL);
                    serialParams.fRtsControl = RTS_CONTROL_TOGGLE; // turns off RTS since there is no TX pending
                    SetCommState( serialHandle, &serialParams );
        isRead = ReadFile(serialHandle, (output), 8, &dwBytesRead, NULL);
        cload=(int_T)(output[5])+(int_T)(output[6])*256;
        return cload ;
    }

int curr_pos(SimStruct *S )
  {
    
        int_T  checksum  ,pos ;
  data[3]=0x04;
  data[4]=0x02;
  data[5]=0x24;
  data[6]=0x02;
  checksum=~(id+4+2+0x24+2);
  data[7]=checksum;
// isWritten = WriteFile(serialHandle,&data,(DWORD) sizeof(data), &dwBytesWritten, NULL);
                serialParams.fRtsControl = RTS_CONTROL_ENABLE; // manual RTS control and turns on RTS
                SetCommState( serialHandle, &serialParams );     
       isWritten = WriteFile(serialHandle, &data ,10, &dwBytesWritten, NULL);
                serialParams.fRtsControl = RTS_CONTROL_TOGGLE; // turns off RTS since there is no TX pending
                 SetCommState( serialHandle, &serialParams );          
        isRead = ReadFile(serialHandle, output,8, &dwBytesRead, NULL);
      pos=((int_T)(output[5])+(int_T)(output[6])*256);  
        return pos;
    }

 void pos_cntr(SimStruct *S ,float a)
  {
    int_T  checksum ,high,low;
    high=(int_T)(a/256);
    low=(int_T)(a-high*256);
    data[3]=0x05;
    data[4]=0x03;
    data[5]=0x1e;
    data[6]=low;
    data[7]=high;
    checksum=~(id+5+3+high+low+0x1e);
    data[8]=checksum; 
         // isWritten = WriteFile(serialHandle,&data,(DWORD) sizeof(data), &dwBytesWritten, NULL);
    
                 serialParams.fRtsControl = RTS_CONTROL_ENABLE; // manual RTS control and turns on RTS
                 SetCommState( serialHandle, &serialParams );    
        isWritten = WriteFile(serialHandle, &data , 10, &dwBytesWritten, NULL);
                 serialParams.fRtsControl = RTS_CONTROL_TOGGLE; // turns off RTS since there is no TX pending
                 SetCommState( serialHandle, &serialParams );
        isRead = ReadFile(serialHandle, output,8, &dwBytesRead, NULL);
       
    }

 void trq_cntr(SimStruct *S ,float a)
  {
    int_T  checksum , high,low;
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
       //    isWritten = WriteFile(serialHandle,&data,(DWORD) sizeof(data), &dwBytesWritten, NULL);

                   serialParams.fRtsControl = RTS_CONTROL_ENABLE; // manual RTS control and turns on RTS
                 SetCommState( serialHandle, &serialParams );    
        isWritten = WriteFile(serialHandle, &data , 10, &dwBytesWritten, NULL);
                 serialParams.fRtsControl = RTS_CONTROL_TOGGLE; // turns off RTS since there is no TX pending
                 SetCommState( serialHandle, &serialParams );
        isRead = ReadFile(serialHandle, output,8, &dwBytesRead, NULL);
    }


/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T a;
    int_T way;
    InputRealPtrsType uPtrs1,uPtrs2;
    real_T *pos,*load,TRQIN,POSIN ,cl;
    uPtrs1=ssGetInputPortRealSignalPtrs(S,0);
    uPtrs2=ssGetInputPortRealSignalPtrs(S,1);
    load=ssGetOutputPortRealSignal(S,0);
    pos=ssGetOutputPortRealSignal(S,1);
    TRQIN=*uPtrs1[0];
    POSIN=*uPtrs2[0];
    way=param_4;
    
  
      //current position
     if(way==0 | way==2 | way==3 | way==5  | way==6  )
      *pos=(real_T)(curr_pos(S))*PI/2048;
    
  
   
     //  current load
    if (way==1 | way==2 | way==4 | way==5  | way==6 ){       
     cl=(real_T)curr_load(S);
          if (cl<1024)
           *load=(real_T)(cl)/1023*6;
        else 
           *load=(real_T)(-cl+1024)/1023*6;
}
    
 //position control
     if (way==3 | way==4 | way==5)
      {
       a=POSIN*2048/PI;
     pos_cntr(S,a);}  
      

     // torque control
    if (way==0 | way==1 | way==2)
     {
          if ((TRQIN>0) & (TRQIN<6))
     a= TRQIN*1023/6;
      else if((TRQIN<0) & (TRQIN>-6))
      a=-TRQIN*1023/6+1024;
     trq_cntr(S,a);}
      
     if(way>6 | way<0)
      {
     ssSetErrorStatus(S,"Invalid control method; available method are as follow: \n0: trq_cnt + curr_pos\n 1: trq_cnt + curr_load \n2: trq_cnt + curr_load + curr_pos \n3: pos_cnt + curr_pos \n4: pos_cnt + curr_load \n5: pos_cnt + curr_load + curr_pos \n6: curr_load + curr_pos (no set point)");
     return; 
     }
   
     
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
     CloseHandle(serialHandle);
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
