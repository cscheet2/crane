#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project endeffector.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	14
#define CUST_BYTE_NUM_IN	20
#define TOT_BYTE_NUM_ROUND_OUT	16
#define TOT_BYTE_NUM_ROUND_IN	20


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		int16_t     roll_velocity;
		int16_t     pitch_velocity;
		int16_t     yaw_velocity;
		int16_t     actuator_1;
		int16_t     actuator_2;
		int16_t     actuator_3;
		int16_t     actuator_4;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		int16_t     roll_position;
		int16_t     pitch_position;
		int16_t     yaw_position;
		int16_t     roll_velocity;
		int16_t     pitch_velocity;
		int16_t     yaw_velocity;
		int16_t     actuator_1;
		int16_t     actuator_2;
		int16_t     actuator_3;
		int16_t     actuator_4;
	}Cust;
} PROCBUFFER_IN;

#endif