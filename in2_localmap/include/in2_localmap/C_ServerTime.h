/*
 * C_ServerTime.h
 *
 *  Created on: 2013-10-10
 *      Author: greensky
 */

#ifndef C_SERVERTIME_H_
#define C_SERVERTIME_H_

class C_ServerTime
{
public:
	void C_Init()
	{
		StartTime = GetLocalTime() - GetLocalTime() % 86400000;
	}
	long GetServerTime()
	{
		return GetLocalTime()-StartTime;
	}
private:
	int64_t GetLocalTime()
	{
		int64_t time = int64_t((ros::Time::now().toSec())*1000.0);
		return time;
	}
	int64_t StartTime;
};


#endif /* C_SERVERTIME_H_ */
