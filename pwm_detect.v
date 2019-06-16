`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Portland State University
// Engineer: Surya Ravikumar
// 
// Create Date: 04/14/2019 04:38:48 PM
// Module Name: pwm_detect
// Project Name:  Project1 
// Target Devices: Nexys A7
// Description: This module takes in RGB signals, and returns high count of the 3 signals
//              and the total period count. Counts are accumulated until total count
//              reaches a predetermined value. The counts are assigned to respective
//              output ports.
//
//////////////////////////////////////////////////////////////////////////////////


module pwm_detect(
    input clk,
    input rst,
    input Red,
    input Green,
    input Blue,
    output [31:0] Red_count,
    output [31:0] Green_count,
    output [31:0] Blue_count,
    output [31:0] Total_count
    );
    
    reg [31:0] max = 32'h008fffff; //predetermined value to count till
    reg [31:0] r_pwm, g_pwm, b_pwm; //local registers to count
    reg [31:0] t_c;     //local register for total count
    reg [31:0] T;       //local register 2 for total count
    
    //assign output ports
    assign Red_count = r_pwm;
    assign Green_count = g_pwm;
    assign Blue_count = b_pwm;
    assign Total_count = t_c;
    
    always @(posedge clk)
    begin
        if(rst) //reset everything on reset
        begin
            r_pwm <= 32'b0;
            g_pwm <= 32'b0;
            b_pwm <= 32'b0;
            t_c <= 32'b0;
        end
        
        else
        begin
            if(T > max) //if total count reaches max value, reset everything
            begin
                r_pwm <= 32'b0;
                g_pwm <= 32'b0;
                b_pwm <= 32'b0;
                t_c <= 32'b0;
            end
            
            else    //if not, then increment counters
            begin   
                t_c <= t_c + 1'b1;  //total counter increment
                                             
                if(Red)     //increment red counter if red signal high
                    r_pwm <= r_pwm + 1'b1;
                else        //retain old value if low
                    r_pwm <= r_pwm;
                
                if(Green)   //increment green counter if green signal high
                    g_pwm <= g_pwm + 1'b1;
                else        //retain old value if low
                    g_pwm <= g_pwm;
                
                if(Blue)    //increment blue counter if blue signal high
                    b_pwm <= b_pwm + 1'b1;
                else        //retain old value if low
                    b_pwm <= b_pwm;
            end               
        end
    end
    
    always @(*)
    begin
        if(rst) //on reset, reset to 0
        begin
            T = 32'b0;
        end  
        
        else
        begin
            T = t_c;  //updates total counter 2 everytime total counter 1 changed - might be overdoing it having 2 counters, but...  
        end
    end
    
endmodule
