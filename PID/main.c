/*PID*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
PID 算法是一种工控领域常见的控制算法，用于闭环反馈控制。有以下两种分类：
增量式
每次周期性计算出的 PID 为增量值，是在上一次控制量的基础上进行的调整。
位置式
每次周期性计算出的 PID 为绝对的数值，是执行机构实际的位置。
我们使用高级语言的思想去实现两种 PID ，做到对于用户来说，调用相同的接口，
内部实现不同的 PID 算法
*/
#define VAL_MAX (100)

enum PID_MODE
{
	PID_INC  = 0,	//增量式
	PID_POS,		//位置式
};

struct PID
{
	enum PID_MODE mode;

	float	kp;
	float	ki;
	float	kd;

	double	targetPoint;
	double	currError;
	double	lastError;
	double	prevError;

	void 	(*init)(struct PID *this, double targetPoint);
	double	(*outputLimit)(struct PID *this, double output);
	void 	(*setParameter)(struct PID *this, float kp, float ki, float kd);
	double	(*calculate)(struct PID *this, double currenPoint);
};

struct PID_INC
{
	struct PID pid;
};

struct PID_POS
{
	struct PID pid;
	double	iSum;
};

static void PID_init(struct PID *this, double targetPoint)
{
	this->targetPoint = targetPoint;
	this->lastError = 0;
	this->prevError = 0;
}

static double PID_outputLimit(struct PID *this, double output)
{
	if(output < 0)
	{
		output = 0;
	}
	else if(output > VAL_MAX)
	{
		output = VAL_MAX;
	}

	return output;
}

static void PID_setParameter(struct PID *this, float kp, float ki, float kd)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void PID_setTargetValue(struct PID *this, double targetPoint)
{
	this->targetPoint = targetPoint;
}

double PID_getTargetValue(struct PID *this)
{
	return this->targetPoint;
}

static double PID_inccalculate(struct PID *this, double currenPoint);

struct PID_INC g_PID_Inc =
{
	.pid = {
		.mode 			= PID_INC,
		.init 			= PID_init,
		.outputLimit 	= PID_outputLimit,
		.setParameter 	= PID_setParameter,
		.calculate 		= PID_inccalculate,
	},
};

static double PID_inccalculate(struct PID *this, double currenPoint)
{
	this->currError = this->targetPoint - currenPoint;

	double out = this->kp * (this->currError - this->lastError) + \
                 this->ki * this->currError + \
                 this->kd * (this->currError - 2*this->lastError + this->prevError);

	this->lastError = this->currError;
	this->prevError = this->lastError;

	if(this->outputLimit)
	{
		out = this->outputLimit(this, out);
	}

	return out;
}

static void PID_PosInit(struct PID *this, double targetPoint);
static double PID_PosCalculate(struct PID *this, double currenPoint);

struct PID_POS g_PID_Pos =
{
	.pid = {
		.mode			= PID_POS,
		.init 			= PID_PosInit,
		.outputLimit	= PID_outputLimit,
		.setParameter	= PID_setParameter,
		.calculate		= PID_PosCalculate,
	},
};

static void PID_PosInit(struct PID *this, double targetPoint)
{
		PID_init(this, targetPoint);
		struct PID_POS *pid_Handle = (struct PID_POS*)this;
		pid_Handle->iSum = 0;
}

static double PID_PosCalculate(struct PID *this, double currenPoint)
{
	struct PID_POS *pid_Handle = (struct PID_POS*)this;

	this->currError = this->targetPoint - currenPoint;
	//this->lastError = this->currError;

	pid_Handle->iSum += this->currError;

	double out = this->kp * this->currError + \
	this->ki * pid_Handle->iSum + \
	this->kd * (this->currError - this->prevError);

	this->prevError = this->currError;

	if(this->outputLimit)
	{
		out = this->outputLimit(this, out);
	}

	return out;
}

#define PID_MODE (1)

struct KernelCtrl
{
    struct PID *dtvPid;
    int flag;
};

static struct KernelCtrl g_m_kernelCtrl;

void Kernel_Ctrl_Init(void)
{
#if PID_MODE
    g_m_kernelCtrl.dtvPid = &g_PID_Inc.pid;
#else
    g_m_kernelCtrl.dtvPid = &g_PID_Pos.pid;
#endif // PID_MODE

}

int main()
{
    //struct PID this;
    static struct PID this;
    float curval = 2.1;
    float out = 0.0;
    Kernel_Ctrl_Init();

    g_m_kernelCtrl.dtvPid->init(&this, 50.0);
    g_m_kernelCtrl.dtvPid->setParameter(&this, 0.3, 0.5, 0);
    while(1)
    {
        curval += 1.3;
        out = g_m_kernelCtrl.dtvPid->calculate(&this, curval);
        if(curval > 125.0)
            break;
        printf("out = %f\n", out);
    }
    printf("Hello world!\n");
    return 0;
}
