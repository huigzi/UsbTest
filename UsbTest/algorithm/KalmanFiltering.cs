using System;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra;

namespace UsbTest.algorithm
{
    public class KalmanFiltering
    {
        private readonly Matrix<double> _kinematicMotionConstraint;
        private readonly Matrix<double> _gk;
        private readonly Matrix<double> _hk;
        private readonly Matrix<double> _rk;

        private readonly double _noiseGain;
        private readonly double _gateFactor;
        private readonly double _thread;

        public Matrix<double> PreVarience { get; set; }
        public Matrix<double> CurrentVariance { get; set; }
        public Matrix<double> PredictVariance { get; set; }

        public Vector<double> CurrentPosition { get; set; }
        public Vector<double> PrePosition { get; set; }
        public Vector<double> PredictPosition { get; set; }

        public int MissCount{ get; set; }
        public int StartFlag { get; set; }
        public int TrackCount { get; set; }

        public KalmanFiltering()
        {
            var vectorBuilder = Vector<double>.Build;
            var matrixBuild = Matrix<double>.Build;

            PreVarience = matrixBuild.DenseDiagonal(6, 6, 8);
            CurrentVariance = matrixBuild.DenseDiagonal(6, 6, 8);
            PredictVariance = matrixBuild.DenseDiagonal(6, 6, 8);

            CurrentPosition = vectorBuilder.Dense(6, 0);
            PredictPosition = vectorBuilder.Dense(6, 0);
            PrePosition = vectorBuilder.Dense(6, 0);

            double[,] constraintTemp =
            {
                {1, 0.012, 7.2E-5, 0, 0, 0},
                {0, 1, 0.012, 0, 0, 0},
                {0, 0, 1, 0, 0, 0},
                {0, 0, 0, 1, 0.012, 7.2E-5},
                {0, 0, 0, 0, 1, 0.012},
                {0, 0, 0, 0, 0, 1}
            };

            _kinematicMotionConstraint = matrixBuild.DenseOfArray(constraintTemp);


            double[,] gkTemp =
            {
                {
                    1.24416000000000e-11, 2.59200000000000e-09, 2.88000000000000e-07, 1.24416000000000e-11,
                    2.59200000000000e-09, 2.88000000000000e-07
                },
                {
                    2.59200000000000e-09, 5.76000000000000e-07, 7.20000000000000e-05, 2.59200000000000e-09,
                    5.76000000000000e-07, 7.20000000000000e-05
                },
                {
                    2.88000000000000e-07, 7.20000000000000e-05, 0.0120000000000000, 2.88000000000000e-07,
                    7.20000000000000e-05, 0.0120000000000000
                },
                {
                    1.24416000000000e-11, 2.59200000000000e-09, 2.88000000000000e-07, 1.24416000000000e-11,
                    2.59200000000000e-09, 2.88000000000000e-07
                },
                {
                    2.59200000000000e-09, 5.76000000000000e-07, 7.20000000000000e-05, 2.59200000000000e-09,
                    5.76000000000000e-07, 7.20000000000000e-05
                },
                {
                    2.88000000000000e-07, 7.20000000000000e-05, 0.0120000000000000, 2.88000000000000e-07,
                    7.20000000000000e-05, 0.0120000000000000
                }
            };

            _gk = matrixBuild.DenseOfArray(gkTemp);

            _rk = matrixBuild.DenseDiagonal(2, 2, 0.5);

            double[,] hkTemple =
            {
                {1, 0, 0, 0, 0, 0}, 
                {0, 0, 0, 1, 0, 0}

            };
            _hk = matrixBuild.DenseOfArray(hkTemple);
    
            _thread = 15000;
            _noiseGain = 100;
            _gateFactor = 4;
            MissCount = 0;
            StartFlag = 0;
            TrackCount = 0;

        }

        public List<TargetTable> TraceTableEstablishment(Matrix<double> testData)
        {
            List<TargetTable> targetTables = new List<TargetTable>();

            for (int i = 1; i < 26; i++)
            {
                for (int j = 1; j < 71; j++)
                {
                    if (testData[j, i] > testData[j - 1, i] &&
                        testData[j, i] > testData[j + 1, i] &&
                        testData[j, i] > testData[j - 1, i - 1] &&
                        testData[j, i] > testData[j + 1, i + 1] &&
                        testData[j, i] > testData[j, i - 1] &&
                        testData[j, i] > testData[j, i + 1] &&
                        testData[j, i] > testData[j - 1, i + 1] &&
                        testData[j, i] > testData[j + 1, i - 1] &&
                        testData[j, i] > _thread
                    )
                    {
                        var sum = testData[j, i] + testData[j + 1, i] + testData[j - 1, i] + testData[j - 1, i - 1] +
                                  testData[j + 1, i + 1] + testData[j, i - 1] + testData[j, i + 1] +
                                  testData[j - 1, i + 1] + testData[j + 1, i - 1];

                        var rangeCorrectionTemp = (testData[j - 1, i] * j + testData[j + 1, i] * (j + 2) +
                                                   testData[j - 1, i - 1] * j + testData[j + 1, i + 1] * (j + 2) +
                                                   testData[j, i - 1] * (j + 1) + testData[j, i + 1] * (j + 1) +
                                                   testData[j - 1, i + 1] * j + testData[j + 1, i - 1] * (j + 2) +
                                                   testData[j, i] * (j + 1)) / sum;

                        var angleCorrectionTemp = (testData[j - 1, i] * (i + 1) + testData[j + 1, i] * (i + 1) +
                                                   testData[j - 1, i - 1] * i + testData[j + 1, i + 1] * (i + 2) +
                                                   testData[j, i - 1] * i + testData[j, i + 1] * (i + 2) +
                                                   testData[j - 1, i + 1] * (i + 2) + testData[j + 1, i - 1] * i +
                                                   testData[j, i] * (i + 1)) / sum;

                        var angleCorrection = (-80 + (angleCorrectionTemp - 1) * 6) / 180 * Math.PI;
                        var rangeCorrection = rangeCorrectionTemp * 0.0215;

                        var targetTable = new TargetTable
                        {
                            TargetX = rangeCorrection * Math.Sin(angleCorrection),
                            TargetY = rangeCorrection * Math.Cos(angleCorrection),
                            TargetEnv = testData[j, i],
                            Distance = 0
                        };

                        targetTables.Add(targetTable);
                    }
                }
            }

            return targetTables;

        }

        public Vector<double> TrackingIni(List<TargetTable> targetTables)
        {
            var vectorBuilder = Vector<double>.Build;
            var matrixBuild = Matrix<double>.Build;

            var systemState = vectorBuilder.Dense(6, 0);

            double sumX = 0;
            double sumY = 0;

            foreach (var target in targetTables)
            {
                sumX += target.TargetX;
                sumY += target.TargetY;
            }

            systemState[0] = sumX / targetTables.Count;
            systemState[3] = sumY / targetTables.Count;

            PreVarience = matrixBuild.DenseDiagonal(6, 6, 8);
            PredictVariance = matrixBuild.DenseDiagonal(6, 6, 8);

            PredictPosition = vectorBuilder.Dense(6, 0);
            PrePosition = vectorBuilder.Dense(6, 0);

            MissCount = 0;
            TrackCount = 0;
            StartFlag = 1;

            return systemState;
        }

        public void Tracking(List<TargetTable> targetTables)
        {

            var vectorBuilder = Vector<double>.Build;
            var matrixBuild = Matrix<double>.Build;

            //预测
            PredictPosition = _kinematicMotionConstraint * PrePosition;
            PredictVariance = _kinematicMotionConstraint * PreVarience * _kinematicMotionConstraint.Transpose() +
                                  _noiseGain * _gk;

            double temp = PredictVariance[0, 0] * PredictVariance[0, 0] + PredictVariance[3, 3] * PredictVariance[3, 3];

            //疑似点可用判断阈值
            double gateThread = Math.Sqrt(temp) * _gateFactor;


            //计算疑似点的dists
            foreach (var target in targetTables)
            {
            
                double distTemp =
                    (PredictPosition[0] - target.TargetX) *
                    (PredictPosition[0] - target.TargetX) +
                    (PredictPosition[3] - target.TargetY) *
                    (PredictPosition[3] - target.TargetY);

                target.Distance = Math.Sqrt(distTemp);
            }

            //寻找符合阈值的并排序
            var sortTargets = from target in targetTables
                              where target.Distance <= gateThread
                              orderby target.Distance descending
                              select target;

            //如果疑似点可以用
            if (sortTargets.Any())
            {
                int k = 0;
                double sumTemp = 0;
                double[] weight = new double[sortTargets.Count()];
                MissCount = 0;

                foreach (var target in sortTargets)
                {
                    weight[k] = target.Distance;
                    k++;
                    sumTemp += target.Distance;
                }

                for (int i = 0; i < weight.Length; i++)
                {
                    weight[i] = weight[i] / sumTemp;
                }

                //将权重升序排列，距离越远，权重越低
                Array.Sort(weight);

                var destPositionTemp = vectorBuilder.Dense(2, 0);

                k = 0;

                foreach (var target in sortTargets)
                {
                    destPositionTemp[0] += weight[k] * target.TargetX;
                    destPositionTemp[1] += weight[k] * target.TargetY;
                }

                var temp2 = _hk * PredictVariance * _hk.Transpose() + _rk;
                var kK = PredictVariance * _hk.Transpose() * temp2.Inverse();

                destPositionTemp[0] = destPositionTemp[0] - PredictPosition[0];
                destPositionTemp[1] = destPositionTemp[1] - PredictPosition[3];

                CurrentPosition = PredictPosition + kK * destPositionTemp;
                CurrentVariance = (matrixBuild.DenseDiagonal(6, 6, 1) - kK * _hk) * PredictVariance;

                PrePosition = CurrentPosition;
                PreVarience = CurrentVariance;

                TrackCount++;
            }
            else
            {
                MissCount++;
                PrePosition = PredictPosition;
                PreVarience = PredictVariance;
            }
        }
    }
}
