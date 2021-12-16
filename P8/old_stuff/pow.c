/* Example using pow by TechOnTheNet.com */

#include <stdio.h>
#include <math.h>

int main(int argc, const char * argv[])
{
    /* Define temporary variables */
    double value1, value2;
    double result;

    /* Assign the values we will use for the pow calculation */
    value1 = 4;
    value2 = 2;

    /* Calculate the result of value1 raised to the power of value2 */
    result = pow(value1, value2);
//    result = pow(4, 2);

    /* Display the result of the calculation */
    printf("%f raised to the power of %f is %f\n", value1, value2, result);

    return 0;
}
