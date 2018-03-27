


typedef struct PID
{
    double SetPoint; 			// 设定目标Desired value
	double Proportion; 			// 比例常数Proportional Const
	double Integral; 			// 积分常数Integral Const
	double Derivative; 			// 微分常数Derivative Const
	double LastError; 			// Error[-1]
	double PrevError; 			// Error[-2]
	double SumError; 			// Sums of Errors
} PID; 
//PID计算部分
double PIDCalc( PID *pp, double NextPoint )
{
	double dError, Error;
	Error = pp->SetPoint - NextPoint; 					// 偏差
	pp->SumError += Error; 								// 积分
	dError = pp->LastError - pp->PrevError;		 		// 当前微分
	pp->PrevError = pp->LastError;
	pp->LastError = Error;
	return (pp->Proportion * Error 						// 比例项
    + pp->Integral * pp->SumError 						// 积分项
	+ pp->Derivative * dError 							// 微分项
	);
}
//Initialize PID Structure
void PIDInit (PID *pp)
{
	memset( pp,0,sizeof(PID));
}

void main(void)
{
	int a;
	PID sPID; 												// PID Control Structure
	double rOut=0; 											// PID Response (Output)
	double rIn; 											// PID Feedback (Input)
	PIDInit ( &sPID ); 										// Initialize Structure
	sPID.Proportion = 0.5; 									// Set PID Coefficients
	sPID.Integral = 0.5;
	sPID.Derivative = 0.0;
	sPID.SetPoint = 2000.0; 								// Set PID Setpoint

	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	for (a=0;a<50;a++)
	{
		rIn = rOut; 										// Read Input
	    rOut = PIDCalc ( &sPID,rIn ); 						// Perform PID Interation
	}
	while(1);
}
