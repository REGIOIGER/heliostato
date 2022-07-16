#include <stdio.h>
#include <math.h>

const double PI = 3.1415926535897932384626433832795028841971;
const double DR = 180 / PI; //degree / radian factor;
const double W = 2 * PI / 365; // earth's mean orbital angular speed in radians/day 
const double WR = PI / 12; // earth's speed of rotation relative to sun (radians/hour) 
double C = -23.45 / DR; // reverse angle of earth's axial tilt in radians 
const double ST = sin(C); // sine of reverse tilt 
const double CT = cos(C); // cosine of reverse tilt 
const double E2 = 2 * 0.0167; // twice earth's orbital eccentricity 
const double SN = 10 * W; // 10 days from December solstice to New Year (Jan 1) 
const double SP = 12 * W; // 12 days from December solstice to perihelion 

double rad2deg(double radians)
{
    return (180.0/PI)*radians;
}

double deg2rad(double degrees)
{
    return (PI/180.0)*degrees;
}

double ang(double x, double y)
{
	double ang;
	if (x > 0)
	  ang = atan(y/x);
	else if (x < 0)
	  ang = atan(y/x) + PI;
	else if (y > 0)
	  ang = 1 * PI / 2;
	else if (y < 0)
	  ang = -1 * PI / 2;
	else
	  ang = 0;

	return ang;
}

void P2C(double AZ, double EL, double *x, double *y, double *z)
{
	double c;
	*z = sin(EL);
	c = cos(EL);
	*x = c * sin(AZ);
	*y = c * cos(AZ);
}

void C2P(double x, double y, double z, double *AZ, double *EL)
{
	*EL = ang(sqrt(x * x + y * y), z);
	*AZ = ang(y, x);
	if (*AZ < 0)
	  *AZ = *AZ + 2 * PI;
}

int main (int argc, char *argv[])
{
	double A, B, SL, D, CL, DC, LD, sAZ, sEL, mAZ, mEL, tAZ, tEL, tX, tY, tZ, sX, sY, sZ, LT, LG;
	int TZN, Mth, Day, Hr, Mn;

	//User input. Note: For brevity, no error checks on user inputs 
	printf("Use negative numbers for directions opposite to those shown.\n\n");	
	printf("Observer's latitude (degrees North): ");
	//scanf("%lf", &LT);
	LT = 21.090368;
	LT = deg2rad(LT);

	printf("Observer's longitude (degrees East): ");
	//scanf("%lf", &LG);
	LG = -101.658543;
	LG = deg2rad(LG);
	
	printf("\nFor target direction of light reflected from mirror:\n");
	printf("Azimuth of target direction (degrees): ");
	//scanf("%lf", &tAZ);
	tAZ = 200;
	tAZ = deg2rad(tAZ);

	printf("Elevation of target direction (degrees): ");
	//scanf("%lf", &tEL);
	tEL = 12;
	tEL = deg2rad(tEL);

	printf("\nTime Zone (+/- hours from GMT/UT): ");
	//scanf("%d", &TZN);
	TZN = -6;

	printf("Month: ");
	//scanf("%d", &Mth);
	Mth = 9;

	printf("Day: ");
	//scanf("%d", &Day);
	Day = 7;

	printf("Hour (24-hr format): ");
	scanf("%d", &Hr);

	printf("Minutes: ");
	scanf("%d", &Mn);

	
	CL = PI / 2 - LT; // co-latitude 
	D = int(30.6 * ((Mth + 9) % 12) + 58.5 + Day) % 365; // day of year (D = 0 on Jan 1)
	A = W * D + SN; // orbit angle since solstice at mean speed 
	B = A + E2 * sin(A - SP); // angle with correction for eccentricity 
	C = (A - atan(tan(B) / CT)) / PI;
	SL = PI * (C - floor(C + .5)); // solar longitude relative to mean position 
	C = ST * cos(B);
	DC = atan(C / sqrt(1 - C * C)); // solar declination (latitude) 
	LD = (Hr - TZN + Mn / 60.0) * WR + SL + LG; // longitude difference 

	P2C(LD, DC, &sX, &sY, &sZ); // polar axis (perpend'r to azimuth plane) 
	C2P(sY, sZ, sX, &sAZ, &sEL); // horizontal axis 
	P2C(sAZ + CL, sEL, &sY, &sZ, &sX); // rotate by co-latitude 
	
	// Sun's position
	C2P(sX, sY, sZ, &sAZ, &sEL); // vertical axis 
  
	printf("\nSun Azimuth:          %.1f degrees\n",rad2deg(sAZ));
	printf("Sun Elevation:        %.1f degrees\n\n",rad2deg(sEL));
	
	// Mirror aim direction
	P2C(tAZ, tEL, &tX, &tY, &tZ); 
	C2P(sX + tX, sY + tY, sZ + tZ, &mAZ, &mEL);
	printf("Mirror aim direction (perpendicular to surface): \n");
	printf("Mirror Azimuth:       %.1f degrees\n",rad2deg(mAZ));
    	printf("Mirror Elevation:     %.1f degrees\n\n",rad2deg(mEL));
	
	return 0;		
}
