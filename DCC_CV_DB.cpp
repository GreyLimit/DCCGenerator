//
//	DCC_CV_DB.h
//	===========
//
//	Define the data structures that capture the higher level
//	meaning of the individual CVs.
//

#include "DCC_CV_DB.h"

//
//	Decoder Logical Configuration Variables.
//	========================================
//
//	The following tables, held in programming memory, capture
//	the primary (and commonly defined) connfiguration variables
//	as supported by all DCC decoders.
//


//
//	The definitions for each of the logical variables
//

//
//	Configuration Variables broken down into logical value
//	fields either as an element of a single CV or distributed
//	across multiple CVs.  A CV element, when not a set of bits
//	from within a single CV, is either spread across multiple
//	CVs or is an array of value across multiple CVs.
//
//	A compounded value (a single value across multiple CVs) is
//	defined below in LSB first order.
//
//	An array of N values is listed index 0 through to N-1.
//
static const CV_Element cv1_06[] PROGMEM	= {{ 1, 7, 0 },{ 0, 0, 0 }};
static const CV_Element cv1_7[] PROGMEM		= {{ 1, 1, 7 },{ 0, 0, 0 }};
static const CV_Element cv2_07[] PROGMEM	= {{ 2, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv3_07[] PROGMEM	= {{ 3, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv4_07[] PROGMEM	= {{ 4, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv5_07[] PROGMEM	= {{ 5, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv6_07[] PROGMEM	= {{ 6, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv7_07[] PROGMEM	= {{ 7, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv8_07[] PROGMEM	= {{ 8, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv9_07[] PROGMEM	= {{ 9, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv10_07[] PROGMEM	= {{ 10, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv11_07[] PROGMEM	= {{ 11, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv12_07[] PROGMEM	= {{ 12, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv17_67[] PROGMEM	= {{ 17, 2, 6 },{ 0, 0, 0 }};
static const CV_Element cv18_07_cv17_05[] PROGMEM = {{ 18, 8, 0 },{ 17, 6, 0 },{ 0, 0, 0 }};
static const CV_Element cv19_06[] PROGMEM	= {{ 19, 7, 0 },{ 0, 0, 0 }};
static const CV_Element cv19_7[] PROGMEM	= {{ 19, 1, 7 },{ 0, 0, 0 }};
static const CV_Element cv23_06[] PROGMEM	= {{ 23, 7, 0 },{ 0, 0, 0 }};
static const CV_Element cv23_7[] PROGMEM	= {{ 23, 1, 7 },{ 0, 0, 0 }};
static const CV_Element cv24_06[] PROGMEM	= {{ 24, 7, 0 },{ 0, 0, 0 }};
static const CV_Element cv24_7[] PROGMEM	= {{ 24, 1, 7 },{ 0, 0, 0 }};
static const CV_Element cv25_07[] PROGMEM	= {{ 25, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv29_0[] PROGMEM	= {{ 29, 1, 0 },{ 0, 0, 0 }};
static const CV_Element cv29_1[] PROGMEM	= {{ 29, 1, 1 },{ 0, 0, 0 }};
static const CV_Element cv29_2[] PROGMEM	= {{ 29, 1, 2 },{ 0, 0, 0 }};
static const CV_Element cv29_3[] PROGMEM	= {{ 29, 1, 3 },{ 0, 0, 0 }};
static const CV_Element cv29_4[] PROGMEM	= {{ 29, 1, 4 },{ 0, 0, 0 }};
static const CV_Element cv29_5[] PROGMEM	= {{ 29, 1, 5 },{ 0, 0, 0 }};
static const CV_Element cv29_7[] PROGMEM	= {{ 29, 1, 7 },{ 0, 0, 0 }};
static const CV_Element cv65_07[] PROGMEM	= {{ 65, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv66_07[] PROGMEM	= {{ 66, 8, 0 },{ 0, 0, 0 }};
static const CV_Element cv67_cv94[] PROGMEM	= {	{ 67, 8, 0 }, { 68, 8, 0 }, { 69, 8, 0 }, { 70, 8, 0 },
							{ 71, 8, 0 }, { 72, 8, 0 }, { 73, 8, 0 }, { 74, 8, 0 },
							{ 75, 8, 0 }, { 76, 8, 0 }, { 77, 8, 0 }, { 78, 8, 0 },
							{ 79, 8, 0 }, { 80, 8, 0 }, { 81, 8, 0 }, { 82, 8, 0 },
							{ 83, 8, 0 }, { 84, 8, 0 }, { 85, 8, 0 }, { 86, 8, 0 },
							{ 87, 8, 0 }, { 88, 8, 0 }, { 89, 8, 0 }, { 90, 8, 0 },
							{ 91, 8, 0 }, { 92, 8, 0 }, { 93, 8, 0 }, { 94, 8, 0 },
							{ 0, 0, 0 }};
static const CV_Element cv95_07[] PROGMEM	= {{ 95, 8, 0 },{ 0, 0, 0 }};

//
//	Declare the standard configuration variable names
//
static const char cv_short_address[] PROGMEM = "short_address";
static const char cv_power_select[] PROGMEM = "power_select";
static const char cv_v_start[] PROGMEM = "v_start";
static const char cv_acceleration[] PROGMEM = "acceleration";
static const char cv_deceleration[] PROGMEM = "deceleration";
static const char cv_v_high[] PROGMEM = "v_high";
static const char cv_v_mid[] PROGMEM = "v_mid";
static const char cv_Manufacturer_Model[] PROGMEM = "Manufacturer_Model";
static const char cv_Manufacturer_ID[] PROGMEM = "Manufacturer_ID";
static const char cv_RESET_EIGHT[] PROGMEM = "RESET8";
static const char cv_PWM_period[] PROGMEM = "PWM_period";
static const char cv_BEMF_cutoff[] PROGMEM = "BEMF_cutoff";
static const char cv_timeout[] PROGMEM = "timeout";
static const char cv_alt_power_source[] PROGMEM = "alt_power_source";
static const char cv_long_address[] PROGMEM = "long_address";
static const char cv_consist_address[] PROGMEM = "consist_address";
static const char cv_consist_direction[] PROGMEM = "consist_direction";
static const char cv_accel_adjust[] PROGMEM = "accel_adjust";
static const char cv_accel_sign[] PROGMEM = "accel_sign";
static const char cv_decel_adjust[] PROGMEM = "decel_adjust";
static const char cv_decel_sign[] PROGMEM = "decel_sign";
static const char cv_alt_speed_table[] PROGMEM = "alt_speed_table";
static const char cv_direction[] PROGMEM = "direction";
static const char cv_light_control[] PROGMEM = "light_control";
static const char cv_power_source[] PROGMEM = "power_source";
static const char cv_bidirectional_comms[] PROGMEM = "bidirectional_comms";
static const char cv_user_speed_table[] PROGMEM = "user_speed_table";
static const char cv_extended_address[] PROGMEM = "extended_address";
static const char cv_decoder_type[] PROGMEM = "decoder_type";
static const char cv_kick_start[] PROGMEM = "kick_start";
static const char cv_forward_trim[] PROGMEM = "forward_trim";
static const char cv_speed_table[] PROGMEM = "speed_table";
static const char cv_reverse_trim[] PROGMEM = "reverse_trim";

//
//	Update sequences required by the standard CVs.
//
static const CV_Update set_short_address[] PROGMEM	= {{ cv17_67, 0 },{ cv1_7, 0 },{ cv29_5, 0 },{ cv18_07_cv17_05, 3 },{ NULL, 0 }};
static const CV_Update set_long_address[] PROGMEM	= {{ cv17_67, 3 },{ cv1_7, 0 },{ cv1_06, 3 },{ cv29_5, 1 },{ NULL, 0 }};

//
//	ZIMO specific Configuration Variabes.
//
//
//	Generic Sound Parameters
//
static const char z_loco_type[] PROGMEM = "z_loco_type";
static const CV_Element z_cv265[] PROGMEM	= {{ 265, 8, 0 },{ 0, 0, 0 }};
//
static const char z_total_vol[] PROGMEM = "z_total_vol";
static const CV_Element z_cv266[] PROGMEM	= {{ 266, 8, 0 },{ 0, 0, 0 }};
//
//	STEAM Sound Parameters
//
static const char z_chuff_freq[] PROGMEM = "z_chuff_freq";
static const CV_Element z_cv267[] PROGMEM	= {{ 267, 8, 0 },{ 0, 0, 0 }};
//
static const char z_cam_sensor[] PROGMEM = "z_cam_sensor";
static const CV_Element z_cv268[] PROGMEM	= {{ 268, 8, 0 },{ 0, 0, 0 }};
//
static const char z_lead_chuff[] PROGMEM = "z_lead_chuff";
static const CV_Element z_cv269[] PROGMEM	= {{ 269, 8, 0 },{ 0, 0, 0 }};
//
static const char z_slow_chuff[] PROGMEM = "z_slow_chuff";
static const CV_Element z_cv270[] PROGMEM	= {{ 270, 8, 0 },{ 0, 0, 0 }};
//
static const char z_fast_chuff[] PROGMEM = "z_fast_chuff";
static const CV_Element z_cv271[] PROGMEM	= {{ 271, 8, 0 },{ 0, 0, 0 }};
//
static const char z_blowoff_duration[] PROGMEM = "z_blowoff_duration";
static const CV_Element z_cv272[] PROGMEM	= {{ 272, 8, 0 },{ 0, 0, 0 }};
//
static const char z_blowoff_delay[] PROGMEM = "z_blowoff_delay";
static const CV_Element z_cv273[] PROGMEM	= {{ 273, 8, 0 },{ 0, 0, 0 }};
//
static const char z_blowoff_shedule[] PROGMEM = "z_blowoff_shedule";
static const CV_Element z_cv274[] PROGMEM	= {{ 274, 8, 0 },{ 0, 0, 0 }};
//
static const char z_slow_chuff_vol[] PROGMEM = "z_slow_chuff_vol";
static const CV_Element z_cv275[] PROGMEM	= {{ 275, 8, 0 },{ 0, 0, 0 }};
//
static const char z_fast_chuff_vol[] PROGMEM = "z_fast_chuff_vol";
static const CV_Element z_cv276[] PROGMEM	= {{ 276, 8, 0 },{ 0, 0, 0 }};
//
static const char z_chuff_vol_adjust[] PROGMEM = "z_chuff_vol_adjust";
static const CV_Element z_cv277[] PROGMEM	= {{ 277, 8, 0 },{ 0, 0, 0 }};
//
//	REACTION Control Parameters
//
static const char z_load_threshold[] PROGMEM = "z_load_threshold";
static const CV_Element z_cv278[] PROGMEM	= {{ 278, 8, 0 },{ 0, 0, 0 }};
//
static const char z_load_reaction[] PROGMEM = "z_load_reaction";
static const CV_Element z_cv279[] PROGMEM	= {{ 279, 8, 0 },{ 0, 0, 0 }};
//
static const char z_load_influence_diesel[] PROGMEM = "z_load_influence_diesel";
static const CV_Element z_cv280[] PROGMEM	= {{ 280, 8, 0 },{ 0, 0, 0 }};
//
static const char z_load_accl_threshold[] PROGMEM = "z_load_accl_threshold";
static const CV_Element z_cv281[] PROGMEM	= {{ 281, 8, 0 },{ 0, 0, 0 }};
//
static const char z_load_accl_duration[] PROGMEM = "z_load_accl_duration";
static const CV_Element z_cv282[] PROGMEM	= {{ 282, 8, 0 },{ 0, 0, 0 }};
//
static const char z_full_accl_vol[] PROGMEM = "z_full_accl_vol";
static const CV_Element z_cv283[] PROGMEM	= {{ 283, 8, 0 },{ 0, 0, 0 }};
//
static const char z_decl_threshold[] PROGMEM = "z_decl_threshold";
static const CV_Element z_cv284[] PROGMEM	= {{ 284, 8, 0 },{ 0, 0, 0 }};
//
static const char z_decl_vol_duration[] PROGMEM = "z_decl_vol_duration";
static const CV_Element z_cv285[] PROGMEM	= {{ 285, 8, 0 },{ 0, 0, 0 }};
//
static const char z_decl_vol[] PROGMEM = "z_decl_vol";
static const CV_Element z_cv286[] PROGMEM	= {{ 286, 8, 0 },{ 0, 0, 0 }};
//
//	BRAKE Noises
//
static const char z_brake_squeal_threshold[] PROGMEM = "z_brake_squeal_threshold";
static const CV_Element z_cv287[] PROGMEM	= {{ 287, 8, 0 },{ 0, 0, 0 }};
//
static const char z_brake_squeal_enabled_after[] PROGMEM = "z_brake_squeal_enabled_after";
static const CV_Element z_cv288[] PROGMEM	= {{ 288, 8, 0 },{ 0, 0, 0 }};
//
//	ELECTRIC motor noises
//
static const char z_thyristor_step_pitch[] PROGMEM = "z_thyristor_step_pitch";
static const CV_Element z_cv289[] PROGMEM	= {{ 289, 8, 0 },{ 0, 0, 0 }};
//
static const char z_thyristor_medium_pitch[] PROGMEM = "z_thyristor_medium_pitch";
static const CV_Element z_cv290[] PROGMEM	= {{ 290, 8, 0 },{ 0, 0, 0 }};
//
static const char z_thyristor_maximum_pitch[] PROGMEM = "z_thyristor_maximum_pitch";
static const CV_Element z_cv291[] PROGMEM	= {{ 291, 8, 0 },{ 0, 0, 0 }};
//
static const char z_thyristor_pitch_inc_speed[] PROGMEM = "z_thyristor_pitch_inc_speed";
static const CV_Element z_cv292[] PROGMEM	= {{ 292, 8, 0 },{ 0, 0, 0 }};
//
static const char z_thyristor_steady_vol[] PROGMEM = "z_thyristor_steady_vol";
static const CV_Element z_cv293[] PROGMEM	= {{ 293, 8, 0 },{ 0, 0, 0 }};
//
static const char z_thyristor_accl_vol[] PROGMEM = "z_thyristor_accl_vol";
static const CV_Element z_cv294[] PROGMEM	= {{ 294, 8, 0 },{ 0, 0, 0 }};
//
static const char z_thyristor_decl_vol[] PROGMEM = "z_thyristor_decl_vol";
static const CV_Element z_cv295[] PROGMEM	= {{ 295, 8, 0 },{ 0, 0, 0 }};
//
static const char z_motor_full_vol[] PROGMEM = "z_motor_full_vol";
static const CV_Element z_cv296[] PROGMEM	= {{ 296, 8, 0 },{ 0, 0, 0 }};
//
static const char z_motor_min_vol_speed[] PROGMEM = "z_motor_min_vol_speed";
static const CV_Element z_cv297[] PROGMEM	= {{ 297, 8, 0 },{ 0, 0, 0 }};
//
static const char z_motor_full_vol_speed[] PROGMEM = "z_motor_full_vol_speed";
static const CV_Element z_cv298[] PROGMEM	= {{ 298, 8, 0 },{ 0, 0, 0 }};
//
static const char z_motor_speed_pitch[] PROGMEM = "z_motor_speed_pitch";
static const CV_Element z_cv299[] PROGMEM	= {{ 299, 8, 0 },{ 0, 0, 0 }};




//
//	Logical Configuration values by name.
//
static const CV_Value cv_variables[] PROGMEM = {
	//
	//	Standard CV definitions and actions
	//
	// Name				Read	Single	Start	End	Location	Enables
	//				Write	Value	Vale	Vale			Change
	//
	{ cv_short_address,		true,	true,	1,	127,	cv1_06,		set_short_address},
	{ cv_power_select,		true,	true,	0,	1,	cv1_7,		NULL		},
	{ cv_v_start,			true,	true,	0,	255,	cv2_07,		NULL		},
	{ cv_acceleration,		true,	true,	0,	255,	cv3_07,		NULL		},
	{ cv_deceleration,		true,	true,	0,	255,	cv4_07,		NULL		},
	{ cv_v_high,			true,	true,	0,	255,	cv5_07,		NULL		},
	{ cv_v_mid,			true,	true,	0,	255,	cv6_07,		NULL		},
	{ cv_Manufacturer_Model,	false,	true,	0,	255,	cv7_07,		NULL		},
	{ cv_Manufacturer_ID,		false,	true,	0,	255,	cv8_07,		NULL		},
	{ cv_RESET_EIGHT,		true,	true,	8,	8,	cv8_07,		NULL		},	// RESET the decoder.
	{ cv_PWM_period,		true,	true,	0,	255,	cv9_07,		NULL		},	// 8 bit FP representation mantissa(0-4), exponent(5-7)
	{ cv_BEMF_cutoff,		true,	true,	0,	255,	cv10_07,	NULL		},
	{ cv_timeout,			true,	true,	0,	255,	cv11_07,	NULL		},
	{ cv_alt_power_source,		true,	true,	0,	255,	cv12_07,	NULL		},	// Bitmap of alt power options (see Appendix B in S.9.2.2)
	{ cv_long_address,		true,	true,	1,	10239,	cv18_07_cv17_05,set_long_address},
	{ cv_consist_address,		true,	true,	0,	127,	cv19_06,	NULL		},	// 0 means "not in a consist"
	{ cv_consist_direction,		true,	true,	0,	1,	cv19_7,		NULL		},	// 0 same as consist, 1 opposite to consist.
	{ cv_accel_adjust,		true,	true,	0,	127,	cv23_06,	NULL		},
	{ cv_accel_sign,		true,	true,	0,	1,	cv23_7,		NULL		},	// 0 add, 1 subtract
	{ cv_decel_adjust,		true,	true,	0,	127,	cv24_06,	NULL		},
	{ cv_decel_sign,		true,	true,	0,	1,	cv24_7,		NULL		},	// 0 add, 1 subtract
	{ cv_alt_speed_table,		true,	true,	0,	255,	cv25_07,	NULL		},
	{ cv_direction,			true,	true,	0,	1,	cv29_0,		NULL		},
	{ cv_light_control,		true,	true,	0,	1,	cv29_1,		NULL		},
	{ cv_power_source,		true,	true,	0,	1,	cv29_2,		NULL		},
	{ cv_bidirectional_comms,	true,	true,	0,	1,	cv29_3,		NULL		},
	{ cv_user_speed_table,		true,	true,	0,	1,	cv29_4,		NULL		},
	{ cv_extended_address,		true,	true,	0,	1,	cv29_5,		NULL		},
	{ cv_decoder_type,		false,	true,	0,	1,	cv29_7,		NULL		},
	{ cv_kick_start,		true,	true,	0,	255,	cv65_07,	NULL		},
	{ cv_forward_trim,		true,	true,	0,	255,	cv66_07,	NULL		},
	{ cv_speed_table,		true,	false,	0,	255,	cv67_cv94,	NULL		},
	{ cv_reverse_trim,		true,	true,	0,	255,	cv95_07,	NULL		},
	//
	//	ZIMO Sound decoder CVs
	//
	// Name				Read	Single	Start	End	Location	Enable
	//				Write	Value	Vale	Vale			Changes
	//
	{ z_loco_type,			true,	true,	0,	255,	z_cv265,	NULL		},
	{ z_total_vol,			true,	true,	0,	255,	z_cv266,	NULL		},
	{ z_chuff_freq,			true,	true,	0,	255,	z_cv267,	NULL		},
	{ z_cam_sensor,			true,	true,	0,	255,	z_cv268,	NULL		},
	{ z_lead_chuff,			true,	true,	0,	255,	z_cv269,	NULL		},
	{ z_slow_chuff,			true,	true,	0,	255,	z_cv270,	NULL		},
	{ z_fast_chuff,			true,	true,	0,	255,	z_cv271,	NULL		},
	{ z_blowoff_duration,		true,	true,	0,	255,	z_cv272,	NULL		},
	{ z_blowoff_delay,		true,	true,	0,	255,	z_cv273,	NULL		},
	{ z_blowoff_shedule,		true,	true,	0,	255,	z_cv274,	NULL		},
	{ z_slow_chuff_vol,		true,	true,	0,	255,	z_cv275,	NULL		},
	{ z_fast_chuff_vol,		true,	true,	0,	255,	z_cv276,	NULL		},
	{ z_chuff_vol_adjust,		true,	true,	0,	255,	z_cv277,	NULL		},
	{ z_load_threshold,		true,	true,	0,	255,	z_cv278,	NULL		},
	{ z_load_reaction,		true,	true,	0,	255,	z_cv279,	NULL		},
	{ z_load_influence_diesel,	true,	true,	0,	255,	z_cv280,	NULL		},
	{ z_load_accl_threshold,	true,	true,	0,	255,	z_cv281,	NULL		},
	{ z_load_accl_duration,		true,	true,	0,	255,	z_cv282,	NULL		},
	{ z_full_accl_vol,		true,	true,	0,	255,	z_cv283,	NULL		},
	{ z_decl_threshold,		true,	true,	0,	255,	z_cv284,	NULL		},
	{ z_decl_vol_duration,		true,	true,	0,	255,	z_cv285,	NULL		},
	{ z_decl_vol,			true,	true,	0,	255,	z_cv286,	NULL		},
	{ z_brake_squeal_threshold,	true,	true,	0,	255,	z_cv287,	NULL		},
	{ z_brake_squeal_enabled_after,	true,	true,	0,	255,	z_cv288,	NULL		},
	{ z_thyristor_step_pitch,	true,	true,	0,	255,	z_cv289,	NULL		},
	{ z_thyristor_medium_pitch,	true,	true,	0,	255,	z_cv290,	NULL		},
	{ z_thyristor_maximum_pitch,	true,	true,	0,	255,	z_cv291,	NULL		},
	{ z_thyristor_pitch_inc_speed,	true,	true,	0,	255,	z_cv292,	NULL		},
	{ z_thyristor_steady_vol,	true,	true,	0,	255,	z_cv293,	NULL		},
	{ z_thyristor_accl_vol,		true,	true,	0,	255,	z_cv294,	NULL		},
	{ z_thyristor_decl_vol,		true,	true,	0,	255,	z_cv295,	NULL		},
	{ z_motor_full_vol,		true,	true,	0,	255,	z_cv296,	NULL		},
	{ z_motor_min_vol_speed,	true,	true,	0,	255,	z_cv297,	NULL		},
	{ z_motor_full_vol_speed,	true,	true,	0,	255,	z_cv298,	NULL		},
	{ z_motor_speed_pitch,		true,	true,	0,	255,	z_cv299,	NULL		},
	//	End of list marker.
	//
	{ NULL,				false,	false,	0,	0,	NULL,		NULL		}
};



//
//	Return a record for a configuration variable in PROGMEM
//
const CV_Value *find_cv_variable( char *name ) {
	const CV_Value *look;
	
	for( look = cv_variables; (char *)progmem_read_address( look->name ) != NIL( char ); look++ ) {
		if( strcmp_P( name, progmem_read_address( look->name )) == 0 ) {
			return( look );
		}
	}
	return( NULL );
}

//
//	Clear a set of CV change records.
//
void clear_cv_change( CV_Change *list, int len ) {
	while( len-- ) {
		list->cv = 0;
		list->mask = 0;
		list->value = 0;
		list++;
	}
}
//
//	Record a pending CV change
//
bool add_cv_change( CV_Change *list, int len, int cv, int b, int v ) {
	while( len-- ) {
		if(( list->cv == cv )||( list->cv == 0 )) {
			list->cv = cv;
			list->mask |= bit( b );
			if( v ) {
				list->value |= bit( b );
			}
			else {
				list->value &= ~bit( b );
			}
			return( true );
		}
		list++;
	}
	return( false );
}

//
//	Push an update of a specific CV Element into the pending changes.
//
bool make_cv_change( CV_Change *list, int len, const CV_Element *cve, word val, bool combined ) {
	int	cv;
	byte	bits,
		lsb;
	
	while( true ) {
		if(!( cv = progmem_read_word( cve->cv ))) break;
		bits = progmem_read_byte( cve->bits );
		lsb = progmem_read_byte( cve->lsb );
		while( bits-- ) {
			if( !add_cv_change( list, len, cv, lsb, ( val & 1 ))) return( false );
			lsb++;
			val >>= 1;
		}
		if( !combined ) break;
		cve++;
	};
	return( true );
}



//
//	EOF
//
