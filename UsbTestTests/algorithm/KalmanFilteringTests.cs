using MathNet.Numerics.Data.Matlab;
using MathNet.Numerics.LinearAlgebra;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using UsbTest.algorithm;

namespace UsbTestTests.algorithm
{
    [TestClass()]
    public class KalmanFilteringTests
    {
        [TestMethod()]
        public void KalmanFilteringTest()
        {
            var vectorBuilder = Vector<double>.Build;
            var matrixBuild = Matrix<double>.Build;

            var resultPositon = vectorBuilder.Dense(6, 0);
            resultPositon[0] = -0.11712893894299961;
            resultPositon[1] = -0.0085413873210193024;
            resultPositon[2] = -0.00034048259997198428;
            resultPositon[3] = 0.47421163145835582;
            resultPositon[4] = -0.00690331375794314;
            resultPositon[5] = -0.00027109000134432605;

            double[,] prevariance =
            {
                {
                    0.242766594755489, 0.0523105449659795, 0.00101097341848172, 7.62628147622445e-09,
                    1.83912407997933e-06, 0.000104585919975806
                },
                {
                    0.0523105449659795, 7.99334660284189, 0.220587578642543, 1.83912407997933e-06, 0.000460038020961047,
                    0.0287783990337413
                },
                {
                    0.00101097341848172, 0.220587578642543, 10.3999959410456, 0.000104585919975806, 0.0287783990337413,
                    2.39999917381918
                },
                {
                    7.62628147622445e-09, 1.83912407997933e-06, 0.000104585919975806, 0.242766594755489,
                    0.0523105449659795, 0.00101097341848172
                },
                {
                    1.83912407997933e-06, 0.000460038020961047, 0.0287783990337413, 0.0523105449659795,
                    7.99334660284189, 0.220587578642543
                },
                {
                    0.000104585919975806, 0.0287783990337413, 2.39999917381918, 0.00101097341848172, 0.220587578642543,
                    10.3999959410456
                }
            };

            var varianceTemp = matrixBuild.DenseOfArray(prevariance);

            var m1 = MatlabReader.ReadAll<double>("testData.mat");

            KalmanFiltering kalmanFiltering = new KalmanFiltering();

            foreach (Matrix<double> testData in m1.Values)
            {
                var result = kalmanFiltering.TraceTableEstablishment(testData);
                kalmanFiltering.PrePosition[0] = -0.0950698731643758;
                kalmanFiltering.PrePosition[1] = 0.00483652853477623;
                kalmanFiltering.PrePosition[2] = 9.61518040733592e-05;
                kalmanFiltering.PrePosition[3] = 0.487928863356650;
                kalmanFiltering.PrePosition[4] = 0.00140521891949567;
                kalmanFiltering.PrePosition[5] = 3.61578884580855e-05;


                kalmanFiltering.PreVarience = varianceTemp;
                kalmanFiltering.Tracking(result);

                Assert.AreEqual(resultPositon, kalmanFiltering.CurrentPosition);
            }
        }
    }
}