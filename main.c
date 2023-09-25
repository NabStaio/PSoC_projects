/******************************************************************************
* File Name: main.c
*
* Version 2.10
*
* Description: 
*  This code allows to calibrate the sensor and print its state and capacitance.
*
*******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability. 
*******************************************************************************/

#include <project.h>
#include <stdio.h>
#include <stdbool.h>

#define SEND_INTERVAL       (1u)


/******************************************************************************
* Function Name: main
*******************************************************************************
*
* Summary:
*  This function:
*  1. Calibrate SW parameters of the sensor
*  2. send through SW_Tx_UART the state and the capacitance (pF)
*
******************************************************************************/

uint32 GetMax(int n, uint32 x[n]);

int main()
{
    uint32 cap = 0; //variable to store measured capacitance
    char8 cap_uart[4]; //string to send capacitance through UART
    //uint32 finger_th = 0; //variable to store finger treshold param
    //uint32 diff_count[1000]; //array that stores all raw counts
    //uint32 diff_calibrate; //variable that take the max raw count
    //uint32 diff; //variable that stores real time raw count
    //uint32 state = 0; //variable to store state
    //uint32 noise;
    //char8 state_uart[4]; //string to send state condition through UART
    //char8 diff_uart[4]; //string to send state condition through UART
    //char8 noise_uart[4]; //string to send state condition through UART
    //uint32 noise_th = 0; //variable to store noise treshold
    //uint32 hysterisis = 0; //variable to store hysterisis
    CapSense_TST_MEASUREMENT_STATUS_ENUM temp; 
    //bool flag = true; //variable to control calibration
    
    CyGlobalIntEnable;
    
    /* Start the SW_Tx_UART Component */
    SW_Tx_UART_Start();
    
    /* Start the CapSense Component*/
    CapSense_Start();
    
    /*Scan all widgets before process them*/
    CapSense_ScanAllWidgets();
    
    for(;;)
    {
        if(CapSense_IsBusy() == CapSense_NOT_BUSY){
            
            /*Process all widgets*/
            CapSense_ProcessAllWidgets();
            
            /*Establish a communication with the tuner to take more accurate measurements*/
            CapSense_RunTuner();
            
            /*Get the capacitance of the sensor*/
            cap = CapSense_GetSensorCapacitance(CapSense_TAXEL_WDGT_ID, CapSense_TAXEL_SNS0_ID, &temp);
            
            /********************CALIBRATION START******************************/
            
            if(CapSense_IsSensorActive(CapSense_TAXEL_WDGT_ID, CapSense_TAXEL_SNS0_ID) /*&& flag == true*/){
                //SW_Tx_UART_PutString("START CALIBRATION ");
                
                //for(int i=0; i<1000; i++){
                    
                    /*Take raw counts values in 1ms*/
                    //diff_count[i] = CapSense_TAXEL_SNS0_DIFF_VALUE;

                    //}
                    /*Get the max signal to set thresholds*/
                    //diff_calibrate = GetMax(1000,diff_count);
            
                    /*Set the finger treshold for the touch event (80% of the signal)*/
                    //finger_th = diff_calibrate * 0.8;
                    //CapSense_SetParam(CapSense_TAXEL_FINGER_TH_VALUE, finger_th);
            
                    /*Set the noise treshold (40% of the signal)*/
                    //noise_th = diff_calibrate * 0.4;
                    //CapSense_SetParam(CapSense_TAXEL_NOISE_TH_VALUE, noise_th);
            
                    /*Set the hysterisis (10% of the signal)*/
                    //hysterisis = diff_calibrate * 0.1;
                    //CapSense_SetParam(CapSense_TAXEL_HYSTERESIS_VALUE, hysterisis);
                    //flag = false;
                    //SW_Tx_UART_PutString(" END CALIBRATION");
                }
            /********************CALIBRATION END******************************/
                
            /*get the real time signal*/
            //noise = CapSense_TAXEL_SNS0_RAW0_VALUE;
            
            
            
            /*Get the touch event (0 No, 1 Yes)*/
            //if(diff<=(finger_th+hysterisis)){
            //    state = 0;
            //}
            
            //else{
            //    state = 1;
            //}
            
            /*convert uint in string to put in UART*/
            //sprintf(state_uart, "%lu", state);
            sprintf(cap_uart, "%lu", cap);
            
            //sprintf(diff_uart, "%lu", diff);
            //sprintf(noise_uart, "%lu", noise);
            
            /*sending state and capacitance with CSV format*/
            //SW_Tx_UART_PutString(state_uart);
            //SW_Tx_UART_PutString(",");
            SW_Tx_UART_PutString(cap_uart);
            //SW_Tx_UART_PutString(diff_uart);
            SW_Tx_UART_PutString(",");
            //SW_Tx_UART_PutString(noise_uart);
            CapSense_ScanAllWidgets();
            
            
        }
        
        
        //SW_Tx_UART_PutCRLF();
        //CyDelay(SEND_INTERVAL);
        
        //SW_Tx_UART_PutCRLF();
        
    }

}

/*function to get the max number in an array*/
uint32 GetMax(int n,uint32 x[n]){
    uint32 max=x[0];

    for(int i=0; i<=n-1; i=i+1) {
      if( x[i]>max ) {
        max=x[i];
      }
    }
    return max;

}

/* [] END OF FILE */
