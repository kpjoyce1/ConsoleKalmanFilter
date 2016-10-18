using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace consoleKalmanFilter
{
    class Program
    {

        public static Random generator = new Random();
        public static float Gaussian(float mean, float stddev)
        {
            double u1 = generator.NextDouble(); //these are uniform(0,1) random doubles
            double u2 = generator.NextDouble();
            double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) *
                         Math.Sin(2.0 * Math.PI * u2); //random normal(0,1)
            double randNormal =
                         mean + stddev * randStdNormal; //random normal(mean,stdDev^2)

            return (float)randNormal;
        }
        static void Main(string[] args)
        {
            /*
             * constant value as the actual state
             * kalman filter on single variable
             * based on http://scipy-cookbook.readthedocs.io/items/KalmanFiltering.html?highlight=kalman
             */

            float Iterations = 5000;

            float[] SensorReadings = new float[(int)Iterations];
            
            float ActualValue = 0;
            float StartValue = ActualValue;
            
            float[] z = new float[(int)Iterations]; //sensor readings


              
            float Q = 0.01f; //process variance

            float[] xhat = new float[(int)Iterations]; //a posteri estimate of x    
            float[] P = new float[(int)Iterations]; //a posteri error estimate
            float[] xhatMinus = new float[(int)Iterations]; //a priori estimate of x
            float[] PMinus = new float[(int)Iterations]; //a priori error estimate
            float[] K = new float[(int)Iterations]; //gain of blending factor

            float R = (float)Math.Pow(0.1, 2); //estimate of measurement variance

            xhat[0] = 0.0f;
            P[0] = 1.0f;
            z[0] = (float)Gaussian(ActualValue, 0.5f);
            float averageDifference = 0;
            for (int i = 1; i < Iterations; i++)
            {
                if(i % 5 == 0)
                    ActualValue += 1f;

                z[i] = (float)Gaussian(ActualValue, 5f);    

                //time update
                xhatMinus[i] = xhat[i - 1];
                PMinus[i] = P[i - 1] + Q;

                //measurement update
                K[i] = PMinus[i] / (PMinus[i] + R);
                xhat[i] = xhatMinus[i] + K[i] * (z[i] - xhatMinus[i]);
                P[i] = (1 - K[i]) * PMinus[i];

                float difference = (float)Math.Abs(ActualValue - xhat[i]);
                averageDifference += difference;
                Console.WriteLine("Difference: {0}",  difference);
            }
            averageDifference /= Iterations;
            Console.WriteLine("Start Value: {0} End Value: {1} Xhat: {2}", StartValue, ActualValue, xhat[xhat.Length - 1]);
            Console.WriteLine("Average Difference: {0}", averageDifference);


            Console.ReadKey();

        }
    }
}
