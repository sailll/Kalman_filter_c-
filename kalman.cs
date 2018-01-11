using System;
using System.Diagnostics.Contracts;
using WindowsFormsApplication3;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using final_kalman;

namespace final_kalman
{
	public static class Utility
	{
		public static int factorial(int n)
		{
			if (n == 1) return n;
			else return n * factorial(n - 1);
		}
	}

	public class State
	{	
		public double x;
		public double y;
		int n;
		public Matrix c_state;
		Matrix p_state;
		public Matrix best_estimate;
		public Matrix measure;
		Matrix cov;

		public double X
		{ get { return this.x;} }

		public double Y
		{ get { return this.y;} }

		public Matrix getCur()
		{
			return this.c_state;
		}

		public Matrix getPre()
		{
			return this.p_state;
		}



		public State(double _x, double _y,int _n)
		{
			this.x = _x;
			this.y = _y;
			this.n = _n;
			this.cov = this.CovMatrix(0.0000001);
			this.cur_state();
		}

		public State(Matrix _cur)
		{
			this.cov = this.CovMatrix(0.0000001);
			this.c_state = _cur;
		}

		public Matrix CovMatrix(double cov)
		{
			Matrix re = new Matrix(n, n);
			for (int i = 0; i < n; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					re[i, j] = cov;
				}
			}
			return re;
		}

		public Matrix getCov()
		{
			return this.cov;
		}

		public void cur_state()
		{
			Matrix re = new Matrix(1, n);

			for (int i = 0; i < n-1; ++i)
			{
				re[0, i] = Math.Pow(x, 2*i + 1);
			}
			re[0, n - 1] = y;
			this.c_state = re;
		}

		public void cal_measure(double _x, double _y)
		{
			Matrix re = new Matrix(1, n);

			for (int i = 0; i<n-1; ++i)
			{
				re[0, i] = Math.Pow(_x, 2* i + 1);
			}
			re[0, n - 1] = _y;
			this.measure = re;
		}

		public void predict(double delta)
		{
			Contract.Ensures(Contract.Result<Matrix>() != null);
			Matrix next = (Prediction_model.pre_matrix(n, delta) * this.c_state.Matrix_tranpose());
			this.p_state = next;
		}

		public void transform_cov(double delta)
		{
			this.cov = Prediction_model.pre_matrix(n,delta) *this.cov*Prediction_model.pre_matrix(n,delta).Matrix_tranpose();
		}

		public void update_cov(Matrix kalman_gain,Matrix H)
		{
			this.cov = this.cov - kalman_gain * H * this.cov;
			
		}
	}

	public static class Prediction_model
	{
		public static Matrix pre_matrix(int n,double delta)
		{
			Matrix pre = new Matrix(n, n);
			for (int i = 0; i < n - 1; ++i)
			{
				for (int j = 0; j < n - 1; ++j)
				{
					if (i == j)
					{
						pre[i, j] = 1.0 + (i) * delta;
					}
					else
					{
						pre[i, j] = 0;
					}
				}
			}
			for (int i = 0; i < n-1; ++i)
			{
				pre[n - 1, i] = Math.Pow(-1.0, i) / Utility.factorial(2 * i + 1);
			}
			return pre;
		}
	}

	public class Measurement_model
	{
		Matrix H_cov;


		public static Matrix map_matrix(int n)
		{
			Matrix re = new Matrix(n, n);
			for (int i = 0; i < n; ++i)
			{
				re[i, i] = 1.0;
			}
			return re;
		}

		public void mea_cov_matrix(int n)
		{
			Matrix re = new Matrix(n, n);
			for (int i = 0; i < n; ++i)
			{
				re[i, i] = 1.0;
			}
			this.H_cov = re;
		}

		public Matrix getH_cov()
		{
			return this.H_cov;
		}

	}

	public class kalman
	{
		
		private int n;
		private double delta;

		public State cur;

		public kalman(int _n,double _delta)
		{
			this.delta = _delta;
			this.n = _n;
            this.cur = new State(0, 0, n);
		}

		public void make_pre(double x,double y)
		{
            cur.x = x;
            cur.y = y;
            cur.cur_state();
			cur.predict(delta);
			cur.transform_cov(delta);
		}

		public void core(double m_x, double m_y)
		{
			cur.cal_measure(m_x, m_y);
			Measurement_model m_m = new Measurement_model();
			m_m.mea_cov_matrix(n);
			Matrix H = m_m.getH_cov();
			Matrix kalman_gain = cur.getCov() * H.Matrix_tranpose() * 
			                        (H * cur.getCov() * H.Matrix_tranpose()).InverseMatrix();
			Matrix tmp = H * cur.getPre();
			cur.best_estimate = cur.getPre() + kalman_gain * (cur.measure.Matrix_tranpose()
			                                                  - H * cur.getPre());
			cur.c_state = cur.best_estimate;
			cur.update_cov(kalman_gain,H);
		}

		public Form1.customPoint conduct(List<Form1.customPoint> pointlist,int it_cnt)
		{
			make_pre(pointlist.ElementAt(it_cnt).x,pointlist.ElementAt(it_cnt).y);
			try
			{
				core(pointlist.ElementAt(it_cnt - 1).x, pointlist.ElementAt(it_cnt - 1).y);
				Console.WriteLine(cur.best_estimate[0,0]+ " "+cur.best_estimate[cur.best_estimate.Row-1,0]);                  
			}
			catch (Exception e)
			{
				Console.WriteLine(e.ToString());
			}
            return new Form1.customPoint(cur.best_estimate[0, 0], cur.best_estimate[cur.best_estimate.Row - 1, 0]);
        }

	}
}
