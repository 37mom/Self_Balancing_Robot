


#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_



void LPfilter(float rawDATA, float *filteredARRAY, float LPFgain);


void LPfilter(float rawDATA, float *filteredARRAY, float LPFgain)
{
	filteredARRAY[0] = rawDATA*LPFgain + filteredARRAY[1]*(1-LPFgain);
	filteredARRAY[1] = filteredARRAY[0];
}

#endif /* LOWPASSFILTER_H_ */