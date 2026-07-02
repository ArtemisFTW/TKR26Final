#include <hrishikesh.h>

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long constrain(long Input, long MinimumValue, long MaximumValue){
	if(Input>MaximumValue) Input=MaximumValue;
	if(Input<MinimumValue) Input=MinimumValue;
	return Input;
}
