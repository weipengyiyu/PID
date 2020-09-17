/*PID*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
PID �㷨��һ�ֹ������򳣼��Ŀ����㷨�����ڱջ��������ơ����������ַ��ࣺ
����ʽ
ÿ�������Լ������ PID Ϊ����ֵ��������һ�ο������Ļ����Ͻ��еĵ�����
λ��ʽ
ÿ�������Լ������ PID Ϊ���Ե���ֵ����ִ�л���ʵ�ʵ�λ�á�
����ʹ�ø߼����Ե�˼��ȥʵ������ PID �����������û���˵��������ͬ�Ľӿڣ�
�ڲ�ʵ�ֲ�ͬ�� PID �㷨
*/

enum PID_MODE
{
	PID_INC,
	PID_POS
};

typedef struct PID
{
	enum PID_MODE mode;

	float		kp;
	float		ki;
	float		kd;

	double		targetPoint;
	double		currentval;
	double		lastval;
	double		last_lastval;
	double		iSum;

	void		(*init)(struct PID *this, float targetPoint, int mode);
	void		(*setParameter)(struct PID *this, float kp, float ki, float kd);
	double		(*claculate)(struct PID *this, float currentPoint);
}pid_ctrl;

static void PID_init(pid_ctrl *this, float targetPoint, int mode)
{
	if(mode == PID_INC)
	{
		this->mode			= mode;
		this->targetPoint 	= targetPoint;
		this->currentval 	= 0;
		this->lastval		= 0;
		this->last_lastval	= 0;
	}
	else if(mode == PID_POS)
	{
		this->mode			= mode;
		this->targetPoint 	= targetPoint;
		this->currentval 	= 0;
		this->lastval		= 0;
		this->last_lastval	= 0;
		this->iSum			= 0;
	}
}

static void PID_setParameter(pid_ctrl *this, float kp, float ki, float kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void PID_setTargetValue(struct PID *this, float targetPoint)
{
	this->targetPoint = targetPoint;
}

double PID_getTargetValue(struct PID *this)
{
	return this->targetPoint;
}

static double PID_calculate(struct PID *this, float currentPoint)
{
	if(this->mode == PID_INC)
	{
		this->currentval = this->targetPoint - currentPoint;

		double out = this->kp * (this->currentval - this->lastval)+\
					 this->ki * this->currentval+\
					 this->kd * (this->currentval - 2*this->lastval + this->last_lastval);

		this->last_lastval = this->lastval;
		this->lastval = this->currentval;

		return out;
	}
	else if(this->mode == PID_POS)
	{
		this->currentval = this->targetPoint - currentPoint;
		this->iSum += this->currentval;

		double out = this->kp * this->currentval+\
					 this->ki * this->iSum+\
					 this->kd * (this->currentval - this->lastval);

		this->lastval = this->currentval;

		return out;
	}
	else
	{
		return -1;
	}

	return -1;
}

static pid_ctrl g_PID =
{
		.init 			= PID_init,
		.setParameter	= PID_setParameter,
		.claculate		= PID_calculate,
};

int main()
{
    //struct PID this;
    pid_ctrl this;
    float curval = 2.1;
    float out = 0.0;

    g_PID.init(&this,50,PID_INC);
    //g_PID.init(&this,50,PID_POS);
    g_PID.setParameter(&this, 0.2,0.5,0);
    while(1)
    {
        curval += 1.3;
        out = g_PID.claculate(&this,curval);
        if(curval > 125.0)
            break;
        printf("out = %f\n", out);
    }
    printf("Hello world!\n");
    return 0;
}




