#include <stdio.h>
#include <math.h>

double g_desiredX = 0;    //Желаемая точка
double const k_pi = 3.14159;  //Некоторые константы
double g_t=0, g_dt = 0.01; //Начальное время и его приращение
double const k_e = 0.05;  //Точность вычислений

double timeOfTransitionProcess = 0;
double overshoot = 0;
double staticError = 0;
double linearIntegral = 0;
double integralofModuleOfError = 0;
double integralOfSquareOfError = 0;
double integralOfWeightedModuleOfError = 0;
double integralOfWeightedSquareOfError = 0;

class ObjectBall {
private:
  double m_x;
  double m_V;
  double m_a;
  double m_alpha;
  double m_aGravitation = 0, m_aFriction = 0, m_aAirResistance = 0;
  double m_xMax = g_desiredX;

public:
  double const k_g = 9.80665;

  ObjectBall(double x, double V, double a) {
    m_x = x;
    m_V = V;
    m_a = a;
  }

  ObjectBall() {
    m_x = -2;
    m_V = 0;
    m_a = 0;
  }

  void calculateParametersOfTheBall () {
    double const k_k = 0.00001, k_R = 0.05;   //Параметры для силы трения
    double const k_C = 0.5, k_densityAir = 1.226, k_densitySteel = 7800;    //Параметры для силы сопротивления воздуха
    int velocitySign;

    m_aGravitation = k_g * sin(m_alpha * k_pi / 180);
    m_aFriction = k_g * (k_k/k_R) * cos(m_alpha * k_pi / 180);
    m_aAirResistance = ((3 * k_C * k_densityAir * pow(m_V, 2))/(2 * k_densitySteel * k_R)) * cos(m_alpha * k_pi / 180);

    if (m_V > 0.0001) { //Расчет ускорения в зависимости от направления скорости
        velocitySign = -1;
    } else if (m_V < -0.0001) {
        velocitySign = 1;
    } else velocitySign = 0;

    m_a = m_aGravitation + velocitySign * m_aFriction + velocitySign * m_aAirResistance;
    m_V += m_a * g_dt;
    m_x += m_V * g_dt;
  }

  double getX() { return m_x;}
  double getV() { return m_V;}
  double getA() { return m_a;}
  double getAlpha() { return m_alpha;}

  void setAlpha(double alpha) {  m_alpha = alpha;}

  void calculateQualityIndicators () {
      if ((fabs(m_x - g_desiredX)) >= k_e) {
          timeOfTransitionProcess = g_t;
      }

      if (m_x > m_xMax) {
          m_xMax = m_x;
      }
      if (fabs(m_x) > k_e) {
        overshoot = fabs(m_xMax - m_x) * 100 / fabs(m_x);
      } else {
        overshoot = m_xMax * 100;
      }

      staticError = g_desiredX - m_x;

      linearIntegral += (m_x - g_desiredX) * g_dt;
      integralofModuleOfError += fabs(m_x - g_desiredX) * g_dt;
      integralOfSquareOfError += pow((m_x - g_desiredX), 2) * g_dt;
      integralOfWeightedModuleOfError += g_t * fabs(m_x - g_desiredX) * g_dt;
      integralOfWeightedSquareOfError += g_t * pow((m_x - g_desiredX), 2) * g_dt;
  }
};

double hardCalculateANewAngle (double x, double V, double a, double alpha) {
  if (fabs(x - g_desiredX) > 0.5) {   //Если далеко
      if (fabs(V) > k_e) {  //Если движется
          if (x < g_desiredX && V < 0 && alpha < 0) {    //Если в другую сторону и планка наклонена не как нужно (Если в нужную сторону, то ничего не меняем)
              alpha = - alpha;
          } else if (x > g_desiredX && V > 0 && alpha > 0) {
              alpha = - alpha;
          }
      } else {    //Если остановился
          if (x < g_desiredX) { //Если левей нужной точки
              alpha = 2;  //То наклоняем по часовой стрелке
          } else {    //Если правей нужной точки
              alpha = -2; //То наклоняем против часовой стрелки
          }
      }
  } else {    //Если близко
      if (fabs(V) > k_e) {  //Если движется
          if ((x < g_desiredX && V > 0) || (x > g_desiredX && V < 0)) {   //Если в нужную сторону
              if (fabs(V) > k_e) {   //Если быстро
                  if (x < g_desiredX && V > 0 && alpha > 0) {    //Если не тот наклон
                      alpha = - alpha;
                  } else if (x > g_desiredX && V < 0 && alpha < 0) {
                      alpha = - alpha;
                  }
              } else {    //Если медленно
                  if (x < g_desiredX && V > 0 && alpha > 0) {
                      alpha = 0;
                  } else if (x > g_desiredX && V < 0 && alpha < 0) {
                      alpha = 0;
                  }
              }
          } else {    //Если в другую сторону
              if (fabs(V) > k_e) {   //Если быстро
                  if (x < g_desiredX && V < 0 && alpha < 0) {    //Если наклон не в ту сторону
                      alpha = - alpha;
                  } else if (x > g_desiredX && V > 0 && alpha > 0) {
                      alpha = - alpha;
                  }
              } else {    //Если медленно
                  if (x < g_desiredX && V < 0 && fabs(a) >= k_e && alpha < 0) {   //Если ускоряется или с постоянной скоростью
                      alpha = -alpha; //- 0.5 * alpha;    //Меняется на противоположный
                  } else if (x < g_desiredX && V < 0 && fabs(a) >= k_e && alpha > 0) {
                      alpha = alpha; //0.5 * alpha;
                  } else if (x > g_desiredX && V > 0 && fabs(a) >= k_e && alpha > 0) {  //Если замедляется
                      alpha = -alpha; //-0.5 * alpha;   //Меняем на маленький противоположный
                  } else if (x > g_desiredX && V > 0 && fabs(a) >= k_e && alpha < 0) {
                      alpha = alpha; //0.5 * alpha;    //Меняем на маленький в ту же сторону
                  } else if (x < g_desiredX && V < 0 && fabs(a) < k_e && alpha < 0) {
                      alpha = - alpha;
                  } else if (x > g_desiredX && V > 0 && fabs(a) < k_e && alpha > 0) {
                      alpha = - alpha;
                  }
              }
          }
      } else {    //Если остановился
          if (fabs(x - g_desiredX) < k_e) {
            alpha = 0;
          } else if (x < g_desiredX) { //Если левей нужной точки
              alpha = 0.1;  //То немного наклоняем по часовой стрелке
          } else if (x > g_desiredX) {    //Если правей нужной точки
              alpha = -0.1; //То немного наклоняем против часовой стрелки
          }
      }
  }

  double const k_viscosity = 0.001;  //Коэффициент для искусственного торможения при большой скорости
  if (fabs(V) > 0.3) {  //0.3
      alpha += - k_viscosity * V;
  }
  return alpha;
}

double lowerCascade (double x) {
    double k = 5;
    double desiredV = (g_desiredX - x) * k;
    return desiredV;
}
double upperCascade (double desiredV, double V, double alpha) {
    double k = 6;
    alpha = (desiredV - V) * k;
    return alpha;
}
double cascadeCalculateANewAngle (double x, double V, double alpha) {
  return alpha = upperCascade(lowerCascade(x), V, alpha);
}


int main()
{
  FILE *fileHardCalculation;
  fileHardCalculation = fopen("/home/anrdey/practice/HC1.dat", "w");  //Подготавливаем файл для записи
  FILE *fileCascadeCalculation;
  fileCascadeCalculation = fopen("/home/anrdey/practice/CC2.dat", "w");
  FILE *fileQualityIndicatorsHardCalculation;
  fileQualityIndicatorsHardCalculation = fopen("/home/anrdey/practice/QIHC1.dat", "w");
  FILE *fileQualityIndicatorsCascadeCalculation;
  fileQualityIndicatorsCascadeCalculation = fopen("/home/anrdey/practice/QICC2.dat", "w");

  ObjectBall ball { -2, 0, 0 };
  ball.setAlpha(3);

  printf("Time,\t\tX,\t\tV,\t\ta,\t\talpha\n%.2lf\t\t%.3lf\t\t%.3lf\t\t%.3lf\t\t%.2lf\n", g_t, ball.getX(), ball.getV(), ball.getA(), ball.getAlpha());   //Начальное положение шарика и скорость
  fprintf(fileHardCalculation, "#Time,\t\tX,\t\tV,\t\ta,\t\talpha\n%.2lf\t\t%.3lf\t\t%.3lf\t\t%.3lf\t\t%.2lf\n", g_t, ball.getX(), ball.getV(), ball.getA(), ball.getAlpha());
  fprintf(fileCascadeCalculation, "#Time,\t\tX,\t\tV,\t\ta,\t\talpha\n%.2lf\t\t%.3lf\t\t%.3lf\t\t%.3lf\t\t%.2lf\n", g_t, ball.getX(), ball.getV(), ball.getA(), ball.getAlpha());

  for(g_t=g_dt; g_t<50; g_t+=g_dt){   //Время изменяется от 0 до 50 с шагом 0,01
      ball.calculateParametersOfTheBall();
      printf("%.2lf\t\t%.3lf\t\t%.3lf\t\t%.3lf\t\t%.2lf\n", g_t, ball.getX(), ball.getV(), ball.getA(), ball.getAlpha());
      fprintf(fileHardCalculation, "%.2lf\t\t%.3lf\t\t%.3lf\t\t%.3lf\t\t%.2lf\n", g_t, ball.getX(), ball.getV(), ball.getA(), ball.getAlpha());
      fprintf(fileCascadeCalculation, "%.2lf\t\t%.3lf\t\t%.3lf\t\t%.3lf\t\t%.2lf\n", g_t, ball.getX(), ball.getV(), ball.getA(), ball.getAlpha());

      ball.calculateQualityIndicators ();

      //ball.setAlpha(hardCalculateANewAngle(ball.getX(), ball.getV(), ball.getA(), ball.getAlpha()));
      ball.setAlpha(cascadeCalculateANewAngle(ball.getX(), ball.getV(), ball.getAlpha()));
  }

  fprintf (fileQualityIndicatorsHardCalculation, "Time \t %.2lf \n", g_t);
  fprintf (fileQualityIndicatorsHardCalculation, "timeOfTransitionProcess \t %.2lf \n", timeOfTransitionProcess);
  fprintf (fileQualityIndicatorsHardCalculation, "overshoot \t %.2lf \n", overshoot);
  fprintf (fileQualityIndicatorsHardCalculation, "staticError \t %.2lf \n", staticError);
  fprintf (fileQualityIndicatorsHardCalculation, "linearIntegral \t %.2lf \n", linearIntegral);
  fprintf (fileQualityIndicatorsHardCalculation, "integralofModuleOfError \t %.2lf \n", integralofModuleOfError);
  fprintf (fileQualityIndicatorsHardCalculation, "integralOfSquareOfError \t %.2lf \n", integralOfSquareOfError);
  fprintf (fileQualityIndicatorsHardCalculation, "integralOfWeightedModuleOfError \t %.2lf \n", integralOfWeightedModuleOfError);
  fprintf (fileQualityIndicatorsHardCalculation, "integralOfWeightedSquareOfError \t %.2lf \n", integralOfWeightedSquareOfError);

  fprintf (fileQualityIndicatorsCascadeCalculation, "Time \t %.2lf \n", g_t);
  fprintf (fileQualityIndicatorsCascadeCalculation, "timeOfTransitionProcess \t %.2lf \n", timeOfTransitionProcess);
  fprintf (fileQualityIndicatorsCascadeCalculation, "overshoot \t %.2lf \n", overshoot);
  fprintf (fileQualityIndicatorsCascadeCalculation, "staticError \t %.2lf \n", staticError);
  fprintf (fileQualityIndicatorsCascadeCalculation, "linearIntegral \t %.2lf \n", linearIntegral);
  fprintf (fileQualityIndicatorsCascadeCalculation, "integralofModuleOfError \t %.2lf \n", integralofModuleOfError);
  fprintf (fileQualityIndicatorsCascadeCalculation, "integralOfSquareOfError \t %.2lf \n", integralOfSquareOfError);
  fprintf (fileQualityIndicatorsCascadeCalculation, "integralOfWeightedModuleOfError \t %.2lf \n", integralOfWeightedModuleOfError);
  fprintf (fileQualityIndicatorsCascadeCalculation, "integralOfWeightedSquareOfError \t %.2lf \n", integralOfWeightedSquareOfError);

  return 0;
}


