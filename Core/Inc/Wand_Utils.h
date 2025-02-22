#ifndef H_WAND_UTILS
#define H_WAND_UTILS

//thresholds for a dtw match
#define DTW_X_THRESHOLD 6500
#define DTW_Y_THRESHOLD 6500
#define DTW_Z_THRESHOLD 6500


#define STATE_BUFFER_SEL ( state==CAPTURE_1 ? &ring_buffer_capture_1 : &ring_buffer_capture_2 )
#define OTHER_BUFFER_SEL ( state==CAPTURE_1 ? &ring_buffer_capture_2 : &ring_buffer_capture_1 )

typedef enum  {
	IDLE,
	INIT,
	CAPTURE_1,
	CAPTURE_2,
	PRINT_AND_COMPARE,
	COMPARE_STATIC,
	ROLLING_COMPARE,
	DEBUG_STATE,
	GET_SPELL,
	HARD_RESET,
	MAX_WAND_STATE
}WAND_STATE;

static const char* wand_state_name_lut[MAX_WAND_STATE] = {
	"IDLE",
	"INIT",
	"CAPTURE_1",
	"CAPTURE_2",
	"PRINT_AND_COMPARE",
	"COMPARE_STATIC",
	"HARD_RESET",
	"MAX_WAND_STATE"
};













#endif