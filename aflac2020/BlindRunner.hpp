//
//  BlindRunner.hpp
//  aflac2020
//
//  Copyright © 2020 Ahiruchan Koubou. All rights reserved.
//

#ifndef BlindRunner_hpp
#define BlindRunner_hpp

#include "aflac_common.hpp"
#include "LineTracer.hpp"
#include <stdio.h>

#define PERIOD_SPEED_CHG 200 * 1000 // Trace message in every 200 ms
#define PROP_NAME_LEN	48	// プロパティー名の最大長
#define NUM_PROPS	13	// プロパティーの個数

struct courseSection {
	char	id[6];
	int32_t sectionEnd;
	double  curvature;
};

// section id starts with L for LineTracer deligaton
//                   with B for SPEED_BLIND
//                   with R to return to LineTracer 
// Note:
//  curve +625, 0.45 makes right angle
//        +650, 0.40
//        +795, 0.30
const struct courseSection courseMap[] = {
	{"Bst00",  755, 0.0},    // the st00 end point used to set d_cv01_midpoint below!!!
	{"Bcv01", 1270, 0.5334},
	{"Bcv01", 1821, 0.5333}, // the cv01 end point used to set d_cv01_midpoint below!!!
	{"Bst02", 2175, 0.0},
	{"Bcv03", 3361,-0.4793},
	{"Bst04", 3902, 0.0},
  	{"Bcv05", 4527,-0.45},
	{"Bst06", 5050, 0.0},
	{"Bcv07", 5676, 0.45},
	{"Bst08", 5951, 0.0},
	{"Bcv09", 6567, 0.45},
	{"Bst10", 6905, 0.0},
	{"Bcv11", 7700, 0.3},
	{"Bst12", 9095, 0.0}, // 9105
	{"Bcv13", 9888,-0.3}, // 9898
	{"Bst14",10770, 0.0}, // 10780
	{"Rcv15",11600,-0.247},
	{"Lcv15",11600,-0.247}
}; // Note: size of this array is given by sizeof(courseMap)/sizeof(*courseMap)
const char sBcv01[] = "Bcv01";
const int32_t d_cv01_midpoint = (755 + 1821) / 2; // <--- this has to agree with st00 and cv01 end points in the course map above!!!

class BlindRunner : public LineTracer {
private:
	int16_t s_trace_counter;
    Motor*	leftMotor;
    Motor*  rightMotor;
    Motor*	tailMotor;
	int		courseMapSize, currentSection, speedChgCnt;
	int32_t d_offset, d_cv01_line_lost, d_cv01_line_found;
	bool	stopping;

	struct property{
		char name[PROP_NAME_LEN];
		int value;
	};
	struct property props[NUM_PROPS];

	int readLine( FILE* file, char* dst, size_t len );
	void readPropFile( const char* filename );
	int getProp( const char* propname );
protected:
public:
    BlindRunner();
    BlindRunner(Motor* lm, Motor* rm, Motor* tm);
    void haveControl();
    void operate(); // method to invoke from the cyclic handler
    ~BlindRunner();
};

#endif /* BlindRunner_hpp */