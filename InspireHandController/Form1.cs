using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using Newtonsoft.Json.Linq;

namespace InspireHandController
{
	public class DoubleBufferedPanel : Panel
	{
		public DoubleBufferedPanel()
		{
			this.DoubleBuffered = true;
			this.SetStyle(ControlStyles.AllPaintingInWmPaint |
						  ControlStyles.UserPaint |
						  ControlStyles.OptimizedDoubleBuffer, true);
			this.UpdateStyles();
		}
	}

	// Tactile sensor grid control
	public class TactileSensorGrid : UserControl
	{
		private int _rows;
		private int _cols;
		private int[] _sensorData;

		public int Rows => _rows;
		public int Cols => _cols;

		public TactileSensorGrid(int rows, int cols)
		{
			_rows = rows;
			_cols = cols;
			_sensorData = new int[rows * cols];
			this.DoubleBuffered = true;
			this.BackColor = Color.Transparent;
			this.Paint += TactileSensorGrid_Paint;
		}

		public void UpdateData(int[] data)
		{
			if (data != null && data.Length == _sensorData.Length)
			{
				Array.Copy(data, _sensorData, data.Length);
				Invalidate();
			}
		}

		public void ClearData()
		{
			Array.Clear(_sensorData, 0, _sensorData.Length);
			Invalidate();
		}

		public int[] GetData()
		{
			return (int[])_sensorData.Clone();
		}

		private void TactileSensorGrid_Paint(object sender, PaintEventArgs e)
		{
			e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;

			int cellWidth = this.Width / _cols;
			int cellHeight = this.Height / _rows;
			int cellSize = Math.Min(cellWidth, cellHeight);

			int gridWidth = cellSize * _cols;
			int gridHeight = cellSize * _rows;
			int startX = (this.Width - gridWidth) / 2;
			int startY = (this.Height - gridHeight) / 2;

			using (var borderPen = new Pen(Color.FromArgb(140, 140, 140), 0.5f))
			{
				for (int row = 0; row < _rows; row++)
				{
					for (int col = 0; col < _cols; col++)
					{
						int index = row * _cols + col;
						int value = _sensorData[index];

						Color cellColor = GetPressureColor(value);

						int x = startX + col * cellSize;
						int y = startY + row * cellSize;

						using (var cellBrush = new SolidBrush(cellColor))
						{
							e.Graphics.FillRectangle(cellBrush, x, y, cellSize, cellSize);
						}
						e.Graphics.DrawRectangle(borderPen, x, y, cellSize, cellSize);
					}
				}
			}
		}

		private Color GetPressureColor(int value)
		{
			value = Math.Max(0, Math.Min(4095, value));
			float normalized = value / 4095f;

			if (normalized < 0.01f)
			{
				return Color.FromArgb(225, 225, 225);
			}
			else if (normalized < 0.33f)
			{
				float t = normalized / 0.33f;
				int r = (int)(225 - 225 * t);
				int g = (int)(225 + 30 * t);
				int b = (int)(225 - 225 * t);
				return Color.FromArgb(r, g, b);
			}
			else if (normalized < 0.66f)
			{
				float t = (normalized - 0.33f) / 0.33f;
				int r = (int)(0 + 255 * t);
				int g = 255;
				int b = 0;
				return Color.FromArgb(r, g, b);
			}
			else
			{
				float t = (normalized - 0.66f) / 0.34f;
				int r = 255;
				int g = (int)(255 - 255 * t);
				int b = 0;
				return Color.FromArgb(r, g, b);
			}
		}
	}

	// Object visualization panel
	public class ObjectVisualizationPanel : Panel
	{
		private List<ContactPoint> _contactPoints = new List<ContactPoint>();
		private object _contactLock = new object();

		// Minimum requirements to render an object
		private const int MIN_CONTACT_POINTS = 5;
		private const float MIN_TOTAL_PRESSURE = 0.5f; // Sum of normalized pressures

		public class ContactPoint
		{
			public float X;
			public float Y;
			public float Pressure;
			public string Source;
		}

		public ObjectVisualizationPanel()
		{
			this.DoubleBuffered = true;
			this.BackColor = Color.FromArgb(15, 15, 20);
			this.Paint += ObjectVisualizationPanel_Paint;
		}

		public void ClearContacts()
		{
			lock (_contactLock)
			{
				_contactPoints.Clear();
			}
			Invalidate();
		}

		public void UpdateFromTactileData(
			int[] littleTip, int[] littlePad,
			int[] ringTip, int[] ringPad,
			int[] middleTip, int[] middlePad,
			int[] indexTip, int[] indexPad,
			int[] thumbTip, int[] thumbPad,
			int[] palm)
		{
			lock (_contactLock)
			{
				_contactPoints.Clear();

				// Little finger - leftmost
				ProcessFingerSensor(littleTip, 3, 3, 0.12f, 0.05f, 0.08f, 0.12f, "LittleTip");
				ProcessFingerSensor(littlePad, 10, 8, 0.10f, 0.18f, 0.10f, 0.28f, "LittlePad");

				// Ring finger
				ProcessFingerSensor(ringTip, 3, 3, 0.28f, 0.03f, 0.08f, 0.12f, "RingTip");
				ProcessFingerSensor(ringPad, 10, 8, 0.26f, 0.16f, 0.10f, 0.28f, "RingPad");

				// Middle finger - center top
				ProcessFingerSensor(middleTip, 3, 3, 0.44f, 0.01f, 0.08f, 0.12f, "MiddleTip");
				ProcessFingerSensor(middlePad, 10, 8, 0.42f, 0.14f, 0.10f, 0.28f, "MiddlePad");

				// Index finger
				ProcessFingerSensor(indexTip, 3, 3, 0.60f, 0.03f, 0.08f, 0.12f, "IndexTip");
				ProcessFingerSensor(indexPad, 10, 8, 0.58f, 0.16f, 0.10f, 0.28f, "IndexPad");

				// Thumb - right side
				ProcessFingerSensor(thumbTip, 3, 3, 0.82f, 0.25f, 0.10f, 0.10f, "ThumbTip");
				ProcessFingerSensor(thumbPad, 12, 8, 0.78f, 0.38f, 0.14f, 0.30f, "ThumbPad");

				// Palm - center bottom
				ProcessPalmSensor(palm, 8, 14, 0.15f, 0.48f, 0.55f, 0.48f);
			}

			Invalidate();
		}

		private void ProcessFingerSensor(int[] data, int rows, int cols,
			float startX, float startY, float width, float height, string source)
		{
			if (data == null || data.Length != rows * cols) return;

			int threshold = 80;

			for (int r = 0; r < rows; r++)
			{
				for (int c = 0; c < cols; c++)
				{
					int pressure = data[r * cols + c];
					if (pressure > threshold)
					{
						float x = startX + (c + 0.5f) / cols * width;
						float y = startY + (r + 0.5f) / rows * height;

						_contactPoints.Add(new ContactPoint
						{
							X = x,
							Y = y,
							Pressure = Math.Min(1f, pressure / 2500f),
							Source = source
						});
					}
				}
			}
		}

		private void ProcessPalmSensor(int[] data, int rows, int cols,
			float startX, float startY, float width, float height)
		{
			if (data == null || data.Length != rows * cols) return;

			int threshold = 80;

			for (int r = 0; r < rows; r++)
			{
				for (int c = 0; c < cols; c++)
				{
					int pressure = data[r * cols + c];
					if (pressure > threshold)
					{
						float x = startX + (c + 0.5f) / cols * width;
						float y = startY + (r + 0.5f) / rows * height;

						_contactPoints.Add(new ContactPoint
						{
							X = x,
							Y = y,
							Pressure = Math.Min(1f, pressure / 2500f),
							Source = "Palm"
						});
					}
				}
			}
		}

		private bool HasSufficientContact(List<ContactPoint> points)
		{
			if (points.Count < MIN_CONTACT_POINTS)
				return false;

			float totalPressure = points.Sum(p => p.Pressure);
			if (totalPressure < MIN_TOTAL_PRESSURE)
				return false;

			return true;
		}

		private void ObjectVisualizationPanel_Paint(object sender, PaintEventArgs e)
		{
			Graphics g = e.Graphics;
			g.SmoothingMode = SmoothingMode.AntiAlias;

			int w = this.Width;
			int h = this.Height;
			int margin = 10;

			// Draw border
			using (Pen borderPen = new Pen(Color.FromArgb(60, 60, 70), 1))
			{
				g.DrawRectangle(borderPen, margin, margin, w - margin * 2, h - margin * 2);
			}

			List<ContactPoint> points;
			lock (_contactLock)
			{
				points = _contactPoints.ToList();
			}

			// Check if we have enough contact to render an object
			if (!HasSufficientContact(points))
			{
				// Show appropriate message
				using (Font font = new Font("Segoe UI", 10))
				{
					string msg;
					if (points.Count == 0)
					{
						msg = "No contact detected";
					}
					else
					{
						msg = "Insufficient contact";
					}

					SizeF size = g.MeasureString(msg, font);
					g.DrawString(msg, font, Brushes.Gray,
						(w - size.Width) / 2, (h - size.Height) / 2);

					// Show contact count if there's some contact
					if (points.Count > 0)
					{
						string subMsg = $"({points.Count} points, need {MIN_CONTACT_POINTS}+)";
						SizeF subSize = g.MeasureString(subMsg, font);
						g.DrawString(subMsg, font, Brushes.DimGray,
							(w - subSize.Width) / 2, (h + size.Height) / 2 + 5);
					}
				}
				return;
			}

			// Calculate drawing area
			float drawX = margin + 5;
			float drawY = margin + 5;
			float drawW = w - margin * 2 - 10;
			float drawH = h - margin * 2 - 10;

			// Draw pressure blobs
			DrawPressureBlobs(g, points, drawX, drawY, drawW, drawH);

			// Draw object outline
			if (points.Count >= 3)
			{
				DrawObjectOutline(g, points, drawX, drawY, drawW, drawH);
			}

			// Draw info
			using (Font font = new Font("Segoe UI", 8))
			{
				float totalPressure = points.Sum(p => p.Pressure);
				string info = $"Contacts: {points.Count} | Pressure: {totalPressure:F1}";
				g.DrawString(info, font, Brushes.LightGray, margin + 5, h - margin - 16);

				// Estimate object size
				if (points.Count >= 2)
				{
					float minX = points.Min(p => p.X);
					float maxX = points.Max(p => p.X);
					float minY = points.Min(p => p.Y);
					float maxY = points.Max(p => p.Y);

					float objWidth = (maxX - minX) * 100;
					float objHeight = (maxY - minY) * 100;

					string sizeInfo = $"Size: {objWidth:F0}% x {objHeight:F0}%";
					g.DrawString(sizeInfo, font, Brushes.LightGray, margin + 5, margin + 5);
				}
			}
		}

		private void DrawPressureBlobs(Graphics g, List<ContactPoint> points,
			float drawX, float drawY, float drawW, float drawH)
		{
			foreach (var point in points)
			{
				float x = drawX + point.X * drawW;
				float y = drawY + point.Y * drawH;

				float baseRadius = 6 + point.Pressure * 12;

				// Draw glow layers
				for (int i = 3; i >= 0; i--)
				{
					float radius = baseRadius + i * 3;
					int alpha = (int)(point.Pressure * 50 / (i + 1));

					Color glowColor = GetPressureColor(point.Pressure, alpha);

					using (SolidBrush brush = new SolidBrush(glowColor))
					{
						g.FillEllipse(brush, x - radius, y - radius, radius * 2, radius * 2);
					}
				}

				// Draw core
				float coreRadius = baseRadius * 0.5f;
				Color coreColor = GetPressureColor(point.Pressure, 220);
				using (SolidBrush brush = new SolidBrush(coreColor))
				{
					g.FillEllipse(brush, x - coreRadius, y - coreRadius, coreRadius * 2, coreRadius * 2);
				}
			}
		}

		private void DrawObjectOutline(Graphics g, List<ContactPoint> points,
			float drawX, float drawY, float drawW, float drawH)
		{
			List<PointF> screenPoints = points.Select(p => new PointF(
				drawX + p.X * drawW,
				drawY + p.Y * drawH
			)).ToList();

			List<PointF> hull = ComputeConvexHull(screenPoints);

			if (hull.Count >= 3)
			{
				// Draw filled shape
				using (SolidBrush fillBrush = new SolidBrush(Color.FromArgb(30, 80, 180, 255)))
				{
					g.FillPolygon(fillBrush, hull.ToArray());
				}

				// Draw outline
				using (Pen outlinePen = new Pen(Color.FromArgb(150, 80, 180, 255), 2))
				{
					outlinePen.LineJoin = LineJoin.Round;
					g.DrawPolygon(outlinePen, hull.ToArray());
				}
			}
		}

		private List<PointF> ComputeConvexHull(List<PointF> points)
		{
			if (points.Count < 3) return points;

			PointF start = points[0];
			foreach (var p in points)
			{
				if (p.Y > start.Y || (p.Y == start.Y && p.X < start.X))
					start = p;
			}

			var sorted = points.OrderBy(p =>
			{
				if (p.X == start.X && p.Y == start.Y) return -4f;
				return (float)Math.Atan2(p.Y - start.Y, p.X - start.X);
			}).ThenBy(p =>
				(p.X - start.X) * (p.X - start.X) + (p.Y - start.Y) * (p.Y - start.Y)
			).ToList();

			Stack<PointF> hull = new Stack<PointF>();

			foreach (var p in sorted)
			{
				while (hull.Count >= 2)
				{
					var top = hull.Pop();
					var nextToTop = hull.Peek();

					float cross = (top.X - nextToTop.X) * (p.Y - nextToTop.Y) -
								  (top.Y - nextToTop.Y) * (p.X - nextToTop.X);

					if (cross > 0)
					{
						hull.Push(top);
						break;
					}
				}
				hull.Push(p);
			}

			return hull.ToList();
		}

		private Color GetPressureColor(float pressure, int alpha)
		{
			pressure = Math.Max(0, Math.Min(1, pressure));

			int r, gr, b;

			if (pressure < 0.25f)
			{
				float t = pressure / 0.25f;
				r = 0;
				gr = (int)(t * 200);
				b = 200;
			}
			else if (pressure < 0.5f)
			{
				float t = (pressure - 0.25f) / 0.25f;
				r = 0;
				gr = 200;
				b = (int)(200 * (1 - t));
			}
			else if (pressure < 0.75f)
			{
				float t = (pressure - 0.5f) / 0.25f;
				r = (int)(255 * t);
				gr = 200;
				b = 0;
			}
			else
			{
				float t = (pressure - 0.75f) / 0.25f;
				r = 255;
				gr = (int)(200 * (1 - t));
				b = 0;
			}

			return Color.FromArgb(alpha, r, gr, b);
		}
	}

	public partial class Form1 : Form
	{
		// Connection
		private TcpClient _tcpClient;
		private NetworkStream _stream;
		private ushort _transactionId = 0;
		private bool _isConnected = false;
		private readonly object _streamLock = new object();
		private readonly SemaphoreSlim _modbusSemaphore = new SemaphoreSlim(1, 1);

		private string _ipAddress = "192.168.11.210";
		private int _port = 6000;

		// Hand tracking
		private UdpClient _udpClient;
		private Process _pythonProcess;
		private volatile bool _isTrackingRunning = false;
		private CancellationTokenSource _trackingCts;
		private volatile bool _handDetected = false;
		private float[][] _handLandmarks;
		private DateTime _lastDataReceived = DateTime.MinValue;

		// Tactile sensor addresses
		private const int TACTILE_LITTLE_TIP = 3000;
		private const int TACTILE_LITTLE_NAIL = 3018;
		private const int TACTILE_LITTLE_PAD = 3210;
		private const int TACTILE_RING_TIP = 3370;
		private const int TACTILE_RING_NAIL = 3388;
		private const int TACTILE_RING_PAD = 3580;
		private const int TACTILE_MIDDLE_TIP = 3740;
		private const int TACTILE_MIDDLE_NAIL = 3758;
		private const int TACTILE_MIDDLE_PAD = 3950;
		private const int TACTILE_INDEX_TIP = 4110;
		private const int TACTILE_INDEX_NAIL = 4128;
		private const int TACTILE_INDEX_PAD = 4320;
		private const int TACTILE_THUMB_TIP = 4480;
		private const int TACTILE_THUMB_NAIL = 4498;
		private const int TACTILE_THUMB_MID = 4690;
		private const int TACTILE_THUMB_PAD = 4708;
		private const int TACTILE_PALM = 4900;

		private bool _isReadingTactile = false;
		private CancellationTokenSource _tactileCts;

		// Angle control
		private short[] _currentAngles = new short[6];
		private short[] _targetAngles = new short[6];
		private volatile short[] _trackedAngles = new short[6];
		private short _globalSpeed = 500;
		private short _globalForce = 1000;

		// Command queue
		private ConcurrentQueue<short[]> _angleCommandQueue = new ConcurrentQueue<short[]>();
		private System.Windows.Forms.Timer _commandProcessTimer;

		// UI Controls
		private TextBox txtIpAddress;
		private NumericUpDown numPort;
		private Button btnConnect;
		private Button btnDisconnect;
		private Label lblStatus;

		private TrackBar trackSpeed;
		private TrackBar trackForce;
		private Label lblSpeed;
		private Label lblForce;

		private Button btnOpenHand;
		private Button btnCloseHand;
		private Button btnStartTracking;
		private Button btnStopTracking;
		private CheckBox chkMirrorControl;

		private DoubleBufferedPanel panelTracking;
		private Panel panelTactile;
		private ObjectVisualizationPanel panelObjectVis;
		private TextBox txtLog;

		private TrackBar[] fingerSliders = new TrackBar[6];
		private Label[] fingerLabels = new Label[6];

		// Tactile sensor grids
		private TactileSensorGrid gridLittleTip, gridLittleNail, gridLittlePad;
		private TactileSensorGrid gridRingTip, gridRingNail, gridRingPad;
		private TactileSensorGrid gridMiddleTip, gridMiddleNail, gridMiddlePad;
		private TactileSensorGrid gridIndexTip, gridIndexNail, gridIndexPad;
		private TactileSensorGrid gridThumbTip, gridThumbNail, gridThumbMid, gridThumbPad;
		private TactileSensorGrid gridPalm;

		private System.Windows.Forms.Timer updateTimer;
		private bool _isUpdatingSliders = false;

		private readonly string[] FINGER_NAMES = { "Little", "Ring", "Middle", "Index", "Thumb Bend", "Thumb Rotate" };

		public Form1()
		{
			InitializeComponent();
			this.DoubleBuffered = true;
			SetupUI();
			SetupTimers();

			for (int i = 0; i < 6; i++)
			{
				_currentAngles[i] = 1000;
				_targetAngles[i] = 1000;
				_trackedAngles[i] = 1000;
			}
		}

		private static int Clamp(int value, int min, int max)
		{
			if (value < min) return min;
			if (value > max) return max;
			return value;
		}

		private static short ClampShort(int value, int min, int max)
		{
			return (short)Clamp(value, min, max);
		}

		private void SetupUI()
		{
			this.Text = "Inspire Robotics Dexterous Hand Controller";
			this.Size = new Size(1480, 920);
			this.MinimumSize = new Size(1400, 870);
			this.StartPosition = FormStartPosition.CenterScreen;
			this.BackColor = Color.FromArgb(245, 245, 250);

			// Connection Group
			GroupBox grpConnection = new GroupBox
			{
				Text = "Connection",
				Location = new Point(10, 10),
				Size = new Size(350, 100)
			};
			this.Controls.Add(grpConnection);

			grpConnection.Controls.Add(new Label
			{
				Text = "IP Address:",
				Location = new Point(10, 25),
				AutoSize = true
			});

			txtIpAddress = new TextBox
			{
				Text = _ipAddress,
				Location = new Point(80, 22),
				Size = new Size(120, 23)
			};
			grpConnection.Controls.Add(txtIpAddress);

			grpConnection.Controls.Add(new Label
			{
				Text = "Port:",
				Location = new Point(210, 25),
				AutoSize = true
			});

			numPort = new NumericUpDown
			{
				Minimum = 1,
				Maximum = 65535,
				Value = _port,
				Location = new Point(250, 22),
				Size = new Size(80, 23)
			};
			grpConnection.Controls.Add(numPort);

			btnConnect = new Button
			{
				Text = "Connect",
				Location = new Point(10, 55),
				Size = new Size(100, 30)
			};
			btnConnect.Click += BtnConnect_Click;
			grpConnection.Controls.Add(btnConnect);

			btnDisconnect = new Button
			{
				Text = "Disconnect",
				Location = new Point(120, 55),
				Size = new Size(100, 30),
				Enabled = false
			};
			btnDisconnect.Click += BtnDisconnect_Click;
			grpConnection.Controls.Add(btnDisconnect);

			lblStatus = new Label
			{
				Text = "● Disconnected",
				Location = new Point(230, 62),
				AutoSize = true,
				ForeColor = Color.Red,
				Font = new Font("Segoe UI", 9, FontStyle.Bold)
			};
			grpConnection.Controls.Add(lblStatus);

			// Speed & Force Group
			GroupBox grpControl = new GroupBox
			{
				Text = "Global Speed & Force",
				Location = new Point(370, 10),
				Size = new Size(350, 100)
			};
			this.Controls.Add(grpControl);

			grpControl.Controls.Add(new Label
			{
				Text = "Speed:",
				Location = new Point(10, 25),
				AutoSize = true
			});

			trackSpeed = new TrackBar
			{
				Minimum = 0,
				Maximum = 1000,
				Value = 500,
				Location = new Point(60, 20),
				Size = new Size(200, 45),
				TickFrequency = 100
			};
			trackSpeed.ValueChanged += TrackSpeed_ValueChanged;
			grpControl.Controls.Add(trackSpeed);

			lblSpeed = new Label
			{
				Text = "500",
				Location = new Point(270, 25),
				AutoSize = true
			};
			grpControl.Controls.Add(lblSpeed);

			grpControl.Controls.Add(new Label
			{
				Text = "Force:",
				Location = new Point(10, 60),
				AutoSize = true
			});

			trackForce = new TrackBar
			{
				Minimum = 0,
				Maximum = 3000,
				Value = 1000,
				Location = new Point(60, 55),
				Size = new Size(200, 45),
				TickFrequency = 300
			};
			trackForce.ValueChanged += TrackForce_ValueChanged;
			grpControl.Controls.Add(trackForce);

			lblForce = new Label
			{
				Text = "1000g",
				Location = new Point(270, 60),
				AutoSize = true
			};
			grpControl.Controls.Add(lblForce);

			// Quick Actions Group
			GroupBox grpActions = new GroupBox
			{
				Text = "Quick Actions",
				Location = new Point(730, 10),
				Size = new Size(500, 100)
			};
			this.Controls.Add(grpActions);

			btnOpenHand = new Button
			{
				Text = "Open Hand",
				Location = new Point(10, 25),
				Size = new Size(100, 30)
			};
			btnOpenHand.Click += BtnOpenHand_Click;
			grpActions.Controls.Add(btnOpenHand);

			btnCloseHand = new Button
			{
				Text = "Close Hand",
				Location = new Point(120, 25),
				Size = new Size(100, 30)
			};
			btnCloseHand.Click += BtnCloseHand_Click;
			grpActions.Controls.Add(btnCloseHand);

			btnStartTracking = new Button
			{
				Text = "Start Tracking",
				Location = new Point(10, 60),
				Size = new Size(100, 30),
				BackColor = Color.FromArgb(0, 120, 200),
				ForeColor = Color.White,
				FlatStyle = FlatStyle.Flat
			};
			btnStartTracking.Click += BtnStartTracking_Click;
			grpActions.Controls.Add(btnStartTracking);

			btnStopTracking = new Button
			{
				Text = "Stop Tracking",
				Location = new Point(120, 60),
				Size = new Size(100, 30),
				Enabled = false
			};
			btnStopTracking.Click += BtnStopTracking_Click;
			grpActions.Controls.Add(btnStopTracking);

			chkMirrorControl = new CheckBox
			{
				Text = "Send Tracking to Hand",
				Location = new Point(240, 30),
				AutoSize = true,
				Checked = false
			};
			grpActions.Controls.Add(chkMirrorControl);

			grpActions.Controls.Add(new Label
			{
				Text = "(Uses Python + MediaPipe)",
				Location = new Point(240, 55),
				AutoSize = true,
				ForeColor = Color.Gray
			});

			// Finger Sliders Group
			GroupBox grpFingers = new GroupBox
			{
				Text = "Individual Finger Control (0 = Closed, 1000 = Open)",
				Location = new Point(10, 120),
				Size = new Size(350, 250)
			};
			this.Controls.Add(grpFingers);

			for (int i = 0; i < 6; i++)
			{
				grpFingers.Controls.Add(new Label
				{
					Text = FINGER_NAMES[i] + ":",
					Location = new Point(10, 25 + i * 35),
					Size = new Size(90, 20)
				});

				TrackBar slider = new TrackBar
				{
					Minimum = 0,
					Maximum = 1000,
					Value = 1000,
					Location = new Point(100, 20 + i * 35),
					Size = new Size(180, 45),
					TickFrequency = 100,
					Tag = i
				};
				slider.ValueChanged += FingerSlider_ValueChanged;
				fingerSliders[i] = slider;
				grpFingers.Controls.Add(slider);

				Label valLbl = new Label
				{
					Text = "1000",
					Location = new Point(290, 25 + i * 35),
					Size = new Size(50, 20)
				};
				fingerLabels[i] = valLbl;
				grpFingers.Controls.Add(valLbl);
			}

			// Tracking Data Display
			GroupBox grpTracking = new GroupBox
			{
				Text = "Hand Tracking Visualization",
				Location = new Point(370, 120),
				Size = new Size(350, 350)
			};
			this.Controls.Add(grpTracking);

			panelTracking = new DoubleBufferedPanel
			{
				Location = new Point(10, 20),
				Size = new Size(330, 320),
				BackColor = Color.FromArgb(20, 20, 25)
			};
			panelTracking.Paint += PanelTracking_Paint;
			grpTracking.Controls.Add(panelTracking);

			// Tactile Sensor Display
			GroupBox grpTactile = new GroupBox
			{
				Text = "Tactile Sensors",
				Location = new Point(730, 120),
				Size = new Size(500, 350)
			};
			this.Controls.Add(grpTactile);

			panelTactile = new Panel
			{
				Location = new Point(10, 20),
				Size = new Size(480, 320),
				BackColor = Color.FromArgb(252, 252, 252),
				BorderStyle = BorderStyle.FixedSingle
			};
			grpTactile.Controls.Add(panelTactile);

			SetupTactileGrids();

			// Object Visualization
			GroupBox grpObjectVis = new GroupBox
			{
				Text = "Object Visualization",
				Location = new Point(1240, 120),
				Size = new Size(220, 350)
			};
			this.Controls.Add(grpObjectVis);

			panelObjectVis = new ObjectVisualizationPanel
			{
				Location = new Point(10, 20),
				Size = new Size(200, 320)
			};
			grpObjectVis.Controls.Add(panelObjectVis);

			// Log
			GroupBox grpLog = new GroupBox
			{
				Text = "Information",
				Location = new Point(10, 480),
				Size = new Size(1450, 390),
				Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom
			};
			this.Controls.Add(grpLog);

			txtLog = new TextBox
			{
				Multiline = true,
				ReadOnly = true,
				ScrollBars = ScrollBars.Vertical,
				Location = new Point(10, 20),
				Size = new Size(1430, 360),
				Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom,
				Font = new Font("Consolas", 9)
			};
			grpLog.Controls.Add(txtLog);

			Log("=== Inspire Robotics Dexterous Hand Controller ===");
			Log("");
			Log("SETUP:");
			Log("1. Connect to the robot hand to view tactile sensors");
			Log("2. For hand tracking: py -3.11 -m pip install opencv-python mediapipe==0.10.9");
			Log("");
		}

		private void SetupTactileGrids()
		{
			int fingerWidth = 55;
			int fingerGap = 8;
			int tipHeight = 25;
			int nailHeight = 65;
			int padHeight = 55;
			int sectionGap = 3;
			int startY = 22;
			int labelHeight = 16;

			int fourFingersWidth = fingerWidth * 4 + fingerGap * 3;
			int startX = 12;

			int[] fingerX = new int[4];
			for (int i = 0; i < 4; i++)
			{
				fingerX[i] = startX + i * (fingerWidth + fingerGap);
			}

			// Little finger
			AddFingerLabel("Little", fingerX[0], startY - labelHeight, fingerWidth);
			gridLittleTip = CreateAndAddGrid(3, 3, fingerX[0], startY, fingerWidth, tipHeight);
			gridLittleNail = CreateAndAddGrid(12, 8, fingerX[0], startY + tipHeight + sectionGap, fingerWidth, nailHeight);
			gridLittlePad = CreateAndAddGrid(10, 8, fingerX[0], startY + tipHeight + nailHeight + sectionGap * 2, fingerWidth, padHeight);

			// Ring finger
			AddFingerLabel("Ring", fingerX[1], startY - labelHeight, fingerWidth);
			gridRingTip = CreateAndAddGrid(3, 3, fingerX[1], startY, fingerWidth, tipHeight);
			gridRingNail = CreateAndAddGrid(12, 8, fingerX[1], startY + tipHeight + sectionGap, fingerWidth, nailHeight);
			gridRingPad = CreateAndAddGrid(10, 8, fingerX[1], startY + tipHeight + nailHeight + sectionGap * 2, fingerWidth, padHeight);

			// Middle finger
			AddFingerLabel("Middle", fingerX[2], startY - labelHeight, fingerWidth);
			gridMiddleTip = CreateAndAddGrid(3, 3, fingerX[2], startY, fingerWidth, tipHeight);
			gridMiddleNail = CreateAndAddGrid(12, 8, fingerX[2], startY + tipHeight + sectionGap, fingerWidth, nailHeight);
			gridMiddlePad = CreateAndAddGrid(10, 8, fingerX[2], startY + tipHeight + nailHeight + sectionGap * 2, fingerWidth, padHeight);

			// Index finger
			AddFingerLabel("Index", fingerX[3], startY - labelHeight, fingerWidth);
			gridIndexTip = CreateAndAddGrid(3, 3, fingerX[3], startY, fingerWidth, tipHeight);
			gridIndexNail = CreateAndAddGrid(12, 8, fingerX[3], startY + tipHeight + sectionGap, fingerWidth, nailHeight);
			gridIndexPad = CreateAndAddGrid(10, 8, fingerX[3], startY + tipHeight + nailHeight + sectionGap * 2, fingerWidth, padHeight);

			// Finger bottom position
			int fingerBottom = startY + tipHeight + nailHeight + padHeight + sectionGap * 2;

			// Palm
			int palmGap = 18;
			int palmY = fingerBottom + palmGap;
			int palmHeight = 107;

			AddFingerLabel("Palm", startX + fourFingersWidth / 2 - 15, palmY - labelHeight + 2, 30);
			gridPalm = CreateAndAddGrid(8, 14, startX, palmY, fourFingersWidth, palmHeight);

			// Thumb
			int thumbX = startX + fourFingersWidth + fingerGap * 2 + 8;
			int thumbStartY = startY + 35;

			int thumbTipHeight = 22;
			int thumbNailHeight = 55;
			int thumbMidHeight = 22;
			int thumbPadHeight = 55;

			AddFingerLabel("Thumb", thumbX, thumbStartY - labelHeight, fingerWidth);
			gridThumbTip = CreateAndAddGrid(3, 3, thumbX, thumbStartY, fingerWidth, thumbTipHeight);
			gridThumbNail = CreateAndAddGrid(12, 8, thumbX, thumbStartY + thumbTipHeight + sectionGap, fingerWidth, thumbNailHeight);
			gridThumbMid = CreateAndAddGrid(3, 3, thumbX, thumbStartY + thumbTipHeight + thumbNailHeight + sectionGap * 2, fingerWidth, thumbMidHeight);
			gridThumbPad = CreateAndAddGrid(12, 8, thumbX, thumbStartY + thumbTipHeight + thumbNailHeight + thumbMidHeight + sectionGap * 3, fingerWidth, thumbPadHeight);

			// Legend
			int legendX = thumbX + fingerWidth + 12;
			int legendY = thumbStartY + 20;
			AddPressureLegend(legendX, legendY);
		}

		private TactileSensorGrid CreateAndAddGrid(int rows, int cols, int x, int y, int width, int height)
		{
			var grid = new TactileSensorGrid(rows, cols)
			{
				Location = new Point(x, y),
				Size = new Size(width, height)
			};
			panelTactile.Controls.Add(grid);
			return grid;
		}

		private void AddFingerLabel(string text, int x, int y, int width)
		{
			var label = new Label
			{
				Text = text,
				Location = new Point(x, y),
				Size = new Size(width, 15),
				TextAlign = ContentAlignment.MiddleCenter,
				Font = new Font("Segoe UI", 8, FontStyle.Bold),
				ForeColor = Color.FromArgb(70, 70, 70)
			};
			panelTactile.Controls.Add(label);
		}

		private void AddPressureLegend(int x, int y)
		{
			var lblTitle = new Label
			{
				Text = "Pressure",
				Location = new Point(x, y),
				AutoSize = true,
				Font = new Font("Segoe UI", 8, FontStyle.Bold)
			};
			panelTactile.Controls.Add(lblTitle);

			var legendPanel = new Panel
			{
				Location = new Point(x, y + 18),
				Size = new Size(25, 120),
				BorderStyle = BorderStyle.FixedSingle
			};
			legendPanel.Paint += (s, e) =>
			{
				using (var brush = new LinearGradientBrush(
					new Point(0, 0), new Point(0, legendPanel.Height),
					Color.Red, Color.FromArgb(225, 225, 225)))
				{
					ColorBlend blend = new ColorBlend(4);
					blend.Colors = new Color[] { Color.Red, Color.Yellow, Color.Green, Color.FromArgb(225, 225, 225) };
					blend.Positions = new float[] { 0f, 0.33f, 0.66f, 1f };
					brush.InterpolationColors = blend;
					e.Graphics.FillRectangle(brush, 0, 0, legendPanel.Width, legendPanel.Height);
				}
			};
			panelTactile.Controls.Add(legendPanel);

			panelTactile.Controls.Add(new Label
			{
				Text = "High",
				Location = new Point(x + 30, y + 18),
				AutoSize = true,
				Font = new Font("Segoe UI", 7)
			});

			panelTactile.Controls.Add(new Label
			{
				Text = "Low",
				Location = new Point(x + 30, y + 125),
				AutoSize = true,
				Font = new Font("Segoe UI", 7)
			});
		}

		private void SetupTimers()
		{
			updateTimer = new System.Windows.Forms.Timer
			{
				Interval = 33
			};
			updateTimer.Tick += UpdateTimer_Tick;
			updateTimer.Start();

			_commandProcessTimer = new System.Windows.Forms.Timer
			{
				Interval = 50
			};
			_commandProcessTimer.Tick += CommandProcessTimer_Tick;
			_commandProcessTimer.Start();
		}

		private async void CommandProcessTimer_Tick(object sender, EventArgs e)
		{
			if (!_isConnected) return;

			short[] latestCommand = null;
			while (_angleCommandQueue.TryDequeue(out var cmd))
			{
				latestCommand = cmd;
			}

			if (latestCommand != null)
			{
				try
				{
					await SetAnglesAsync(latestCommand);
				}
				catch (Exception ex)
				{
					System.Diagnostics.Debug.WriteLine($"Command process error: {ex.Message}");
				}
			}
		}

		private void Log(string message)
		{
			if (InvokeRequired)
			{
				try { BeginInvoke(new Action<string>(Log), message); } catch { }
				return;
			}
			if (txtLog != null && !txtLog.IsDisposed)
			{
				txtLog.AppendText("[" + DateTime.Now.ToString("HH:mm:ss") + "] " + message + Environment.NewLine);
			}
		}

		// ============== CONNECTION ==============
		private async void BtnConnect_Click(object sender, EventArgs e)
		{
			try
			{
				_ipAddress = txtIpAddress.Text;
				_port = (int)numPort.Value;

				Log("Connecting to " + _ipAddress + ":" + _port + "...");

				_tcpClient = new TcpClient
				{
					ReceiveTimeout = 3000,
					SendTimeout = 3000
				};

				await _tcpClient.ConnectAsync(_ipAddress, _port);
				_stream = _tcpClient.GetStream();
				_isConnected = true;

				lblStatus.Text = "● Connected";
				lblStatus.ForeColor = Color.Green;
				btnConnect.Enabled = false;
				btnDisconnect.Enabled = true;

				Log("Connected successfully!");

				await SetGlobalSpeedAsync(_globalSpeed);
				await SetGlobalForceAsync(_globalForce);

				StartTactileReading();
			}
			catch (Exception ex)
			{
				Log("Connection failed: " + ex.Message);
				MessageBox.Show("Connection failed: " + ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
			}
		}

		private void BtnDisconnect_Click(object sender, EventArgs e)
		{
			Disconnect();
		}

		private void Disconnect()
		{
			try
			{
				StopTactileReading();

				_isConnected = false;
				if (_stream != null)
				{
					_stream.Close();
					_stream = null;
				}
				if (_tcpClient != null)
				{
					_tcpClient.Close();
					_tcpClient = null;
				}

				lblStatus.Text = "● Disconnected";
				lblStatus.ForeColor = Color.Red;
				btnConnect.Enabled = true;
				btnDisconnect.Enabled = false;

				ClearAllTactileGrids();

				Log("Disconnected.");
			}
			catch (Exception ex)
			{
				Log("Disconnect error: " + ex.Message);
			}
		}

		private void ClearAllTactileGrids()
		{
			gridLittleTip?.ClearData();
			gridLittleNail?.ClearData();
			gridLittlePad?.ClearData();
			gridRingTip?.ClearData();
			gridRingNail?.ClearData();
			gridRingPad?.ClearData();
			gridMiddleTip?.ClearData();
			gridMiddleNail?.ClearData();
			gridMiddlePad?.ClearData();
			gridIndexTip?.ClearData();
			gridIndexNail?.ClearData();
			gridIndexPad?.ClearData();
			gridThumbTip?.ClearData();
			gridThumbNail?.ClearData();
			gridThumbMid?.ClearData();
			gridThumbPad?.ClearData();
			gridPalm?.ClearData();

			panelObjectVis?.ClearContacts();
		}

		// ============== MODBUS TCP ==============
		private async Task<int[]> ReadHoldingRegistersAsync(int startAddress, int quantity)
		{
			if (!_isConnected || _stream == null)
				throw new InvalidOperationException("Not connected");

			await _modbusSemaphore.WaitAsync();
			try
			{
				ushort currentTransactionId;

				lock (_streamLock)
				{
					_transactionId++;
					currentTransactionId = _transactionId;
				}

				byte[] request = new byte[12];
				request[0] = (byte)(currentTransactionId >> 8);
				request[1] = (byte)(currentTransactionId & 0xFF);
				request[2] = 0x00;
				request[3] = 0x00;
				request[4] = 0x00;
				request[5] = 0x06;
				request[6] = 0xFF;
				request[7] = 0x03;
				request[8] = (byte)(startAddress >> 8);
				request[9] = (byte)(startAddress & 0xFF);
				request[10] = (byte)(quantity >> 8);
				request[11] = (byte)(quantity & 0xFF);

				await _stream.WriteAsync(request, 0, request.Length);

				byte[] header = new byte[9];
				int totalRead = 0;
				while (totalRead < 9)
				{
					int bytesRead = await _stream.ReadAsync(header, totalRead, 9 - totalRead);
					if (bytesRead == 0) throw new Exception("Connection closed");
					totalRead += bytesRead;
				}

				int byteCount = header[8];
				byte[] data = new byte[byteCount];
				totalRead = 0;
				while (totalRead < byteCount)
				{
					int bytesRead = await _stream.ReadAsync(data, totalRead, byteCount - totalRead);
					if (bytesRead == 0) throw new Exception("Connection closed");
					totalRead += bytesRead;
				}

				int[] registers = new int[quantity];
				for (int i = 0; i < quantity && i * 2 + 1 < data.Length; i++)
				{
					registers[i] = (data[i * 2] << 8) | data[i * 2 + 1];
				}

				return registers;
			}
			finally
			{
				_modbusSemaphore.Release();
			}
		}

		private async Task WriteMultipleRegistersAsync(int startAddress, int[] values)
		{
			if (!_isConnected || _stream == null) return;

			await _modbusSemaphore.WaitAsync();
			try
			{
				ushort currentTransactionId;
				lock (_streamLock)
				{
					_transactionId++;
					currentTransactionId = _transactionId;
				}

				int quantity = values.Length;
				byte byteCount = (byte)(quantity * 2);

				byte[] request = new byte[13 + byteCount];
				request[0] = (byte)(currentTransactionId >> 8);
				request[1] = (byte)(currentTransactionId & 0xFF);
				request[2] = 0x00;
				request[3] = 0x00;
				request[4] = (byte)((7 + byteCount) >> 8);
				request[5] = (byte)((7 + byteCount) & 0xFF);
				request[6] = 0xFF;
				request[7] = 0x10;
				request[8] = (byte)(startAddress >> 8);
				request[9] = (byte)(startAddress & 0xFF);
				request[10] = (byte)(quantity >> 8);
				request[11] = (byte)(quantity & 0xFF);
				request[12] = byteCount;

				for (int i = 0; i < quantity; i++)
				{
					request[13 + i * 2] = (byte)(values[i] >> 8);
					request[14 + i * 2] = (byte)(values[i] & 0xFF);
				}

				await _stream.WriteAsync(request, 0, request.Length);
				byte[] response = new byte[12];
				await _stream.ReadAsync(response, 0, 12);
			}
			finally
			{
				_modbusSemaphore.Release();
			}
		}

		private async Task SetAnglesAsync(short[] angles)
		{
			if (!_isConnected) return;
			try
			{
				int[] values = new int[6];
				for (int i = 0; i < 6; i++)
					values[i] = ClampShort(angles[i], 0, 1000);
				await WriteMultipleRegistersAsync(1486, values);
			}
			catch (Exception ex)
			{
				System.Diagnostics.Debug.WriteLine("Error setting angles: " + ex.Message);
			}
		}

		private async Task SetGlobalSpeedAsync(short speed)
		{
			if (!_isConnected) return;
			try
			{
				int[] values = new int[6];
				for (int i = 0; i < 6; i++)
					values[i] = ClampShort(speed, 0, 1000);
				await WriteMultipleRegistersAsync(1522, values);
				Log("Speed set to " + speed);
			}
			catch (Exception ex)
			{
				Log("Error setting speed: " + ex.Message);
			}
		}

		private async Task SetGlobalForceAsync(short force)
		{
			if (!_isConnected) return;
			try
			{
				int[] values = new int[6];
				for (int i = 0; i < 6; i++)
					values[i] = ClampShort(force, 0, 3000);
				await WriteMultipleRegistersAsync(1498, values);
				Log("Force set to " + force + "g");
			}
			catch (Exception ex)
			{
				Log("Error setting force: " + ex.Message);
			}
		}

		// ============== TACTILE SENSOR READING ==============
		private void StartTactileReading()
		{
			if (_isReadingTactile) return;

			_isReadingTactile = true;
			_tactileCts = new CancellationTokenSource();

			Task.Run(() => TactileReadLoop(_tactileCts.Token));
			Log("Tactile sensor reading started");
		}

		private void StopTactileReading()
		{
			_isReadingTactile = false;
			if (_tactileCts != null)
			{
				_tactileCts.Cancel();
				_tactileCts = null;
			}
		}

		private async Task TactileReadLoop(CancellationToken token)
		{
			while (_isReadingTactile && !token.IsCancellationRequested && _isConnected)
			{
				try
				{
					var littleTip = await ReadTactileDataAsync(TACTILE_LITTLE_TIP, 9);
					var littleNail = await ReadTactileDataAsync(TACTILE_LITTLE_NAIL, 96);
					var littlePad = await ReadTactileDataAsync(TACTILE_LITTLE_PAD, 80);

					var ringTip = await ReadTactileDataAsync(TACTILE_RING_TIP, 9);
					var ringNail = await ReadTactileDataAsync(TACTILE_RING_NAIL, 96);
					var ringPad = await ReadTactileDataAsync(TACTILE_RING_PAD, 80);

					var middleTip = await ReadTactileDataAsync(TACTILE_MIDDLE_TIP, 9);
					var middleNail = await ReadTactileDataAsync(TACTILE_MIDDLE_NAIL, 96);
					var middlePad = await ReadTactileDataAsync(TACTILE_MIDDLE_PAD, 80);

					var indexTip = await ReadTactileDataAsync(TACTILE_INDEX_TIP, 9);
					var indexNail = await ReadTactileDataAsync(TACTILE_INDEX_NAIL, 96);
					var indexPad = await ReadTactileDataAsync(TACTILE_INDEX_PAD, 80);

					var thumbTip = await ReadTactileDataAsync(TACTILE_THUMB_TIP, 9);
					var thumbNail = await ReadTactileDataAsync(TACTILE_THUMB_NAIL, 96);
					var thumbMid = await ReadTactileDataAsync(TACTILE_THUMB_MID, 9);
					var thumbPad = await ReadTactileDataAsync(TACTILE_THUMB_PAD, 96);

					var palmRaw = await ReadTactileDataAsync(TACTILE_PALM, 112);
					var palmRemapped = RemapPalmData(palmRaw);

					if (!token.IsCancellationRequested)
					{
						try
						{
							this.BeginInvoke(new Action(() =>
							{
								gridLittleTip?.UpdateData(littleTip);
								gridLittleNail?.UpdateData(littleNail);
								gridLittlePad?.UpdateData(littlePad);

								gridRingTip?.UpdateData(ringTip);
								gridRingNail?.UpdateData(ringNail);
								gridRingPad?.UpdateData(ringPad);

								gridMiddleTip?.UpdateData(middleTip);
								gridMiddleNail?.UpdateData(middleNail);
								gridMiddlePad?.UpdateData(middlePad);

								gridIndexTip?.UpdateData(indexTip);
								gridIndexNail?.UpdateData(indexNail);
								gridIndexPad?.UpdateData(indexPad);

								gridThumbTip?.UpdateData(thumbTip);
								gridThumbNail?.UpdateData(thumbNail);
								gridThumbMid?.UpdateData(thumbMid);
								gridThumbPad?.UpdateData(thumbPad);

								gridPalm?.UpdateData(palmRemapped);

								// Update object visualization
								panelObjectVis?.UpdateFromTactileData(
									littleTip, littlePad,
									ringTip, ringPad,
									middleTip, middlePad,
									indexTip, indexPad,
									thumbTip, thumbPad,
									palmRemapped
								);
							}));
						}
						catch { }
					}

					await Task.Delay(100, token);
				}
				catch (OperationCanceledException)
				{
					break;
				}
				catch (Exception ex)
				{
					System.Diagnostics.Debug.WriteLine($"Tactile read error: {ex.Message}");
					await Task.Delay(500, token);
				}
			}
		}

		private async Task<int[]> ReadTactileDataAsync(int startAddress, int count)
		{
			const int maxRegistersPerRead = 100;
			int[] result = new int[count];
			int registersRead = 0;

			while (registersRead < count)
			{
				int registersToRead = Math.Min(maxRegistersPerRead, count - registersRead);
				int[] data = await ReadHoldingRegistersAsync(startAddress + registersRead, registersToRead);

				for (int i = 0; i < registersToRead && i < data.Length; i++)
				{
					result[registersRead + i] = data[i];
				}

				registersRead += registersToRead;
			}

			return result;
		}

		private int[] RemapPalmData(int[] rawData)
		{
			int rows = 8;
			int cols = 14;
			int[] remapped = new int[rows * cols];

			for (int col = 0; col < cols; col++)
			{
				for (int row = 0; row < rows; row++)
				{
					int sourceIndex = col * rows + (rows - 1 - row);
					int destRow = rows - 1 - row;
					int destCol = col;
					int destIndex = destRow * cols + destCol;

					if (sourceIndex < rawData.Length && destIndex < remapped.Length)
					{
						remapped[destIndex] = rawData[sourceIndex];
					}
				}
			}

			return remapped;
		}

		// ============== UI EVENTS ==============
		private async void TrackSpeed_ValueChanged(object sender, EventArgs e)
		{
			_globalSpeed = (short)trackSpeed.Value;
			lblSpeed.Text = _globalSpeed.ToString();
			if (_isConnected) await SetGlobalSpeedAsync(_globalSpeed);
		}

		private async void TrackForce_ValueChanged(object sender, EventArgs e)
		{
			_globalForce = (short)trackForce.Value;
			lblForce.Text = _globalForce + "g";
			if (_isConnected) await SetGlobalForceAsync(_globalForce);
		}

		private async void BtnOpenHand_Click(object sender, EventArgs e)
		{
			Log("Opening hand...");
			for (int i = 0; i < 6; i++)
			{
				_targetAngles[i] = 1000;
				fingerSliders[i].Value = 1000;
			}
			await SetAnglesAsync(_targetAngles);
		}

		private async void BtnCloseHand_Click(object sender, EventArgs e)
		{
			Log("Closing hand...");
			for (int i = 0; i < 5; i++) _targetAngles[i] = 0;
			_targetAngles[5] = 500;
			for (int i = 0; i < 6; i++)
				fingerSliders[i].Value = _targetAngles[i];
			await SetAnglesAsync(_targetAngles);
		}

		private async void FingerSlider_ValueChanged(object sender, EventArgs e)
		{
			if (_isUpdatingSliders) return;
			TrackBar slider = sender as TrackBar;
			int idx = (int)slider.Tag;
			_targetAngles[idx] = (short)slider.Value;
			fingerLabels[idx].Text = slider.Value.ToString();
			if (_isConnected && !chkMirrorControl.Checked)
				await SetAnglesAsync(_targetAngles);
		}

		// ============== HAND TRACKING ==============
		private void BtnStartTracking_Click(object sender, EventArgs e)
		{
			StartTracking();
		}

		private void BtnStopTracking_Click(object sender, EventArgs e)
		{
			StopTracking();
		}

		private void StartTracking()
		{
			try
			{
				_udpClient = new UdpClient(5065);
				_udpClient.Client.ReceiveTimeout = 100;
				_isTrackingRunning = true;
				_trackingCts = new CancellationTokenSource();

				btnStartTracking.Enabled = false;
				btnStopTracking.Enabled = true;

				Log("UDP listener started on port 5065");

				Task.Run(() => UdpReceiveLoop(_trackingCts.Token));
				StartPythonTracker();
			}
			catch (Exception ex)
			{
				Log("Error starting tracking: " + ex.Message);
				btnStartTracking.Enabled = true;
				btnStopTracking.Enabled = false;
			}
		}

		private void StartPythonTracker()
		{
			try
			{
				string scriptPath = System.IO.Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "hand_tracker.py");

				if (!System.IO.File.Exists(scriptPath))
				{
					Log("ERROR: hand_tracker.py not found!");
					Log("Please save hand_tracker.py to: " + AppDomain.CurrentDomain.BaseDirectory);
					return;
				}

				Log("Starting Python hand tracker...");

				ProcessStartInfo psi = new ProcessStartInfo
				{
					FileName = "py",
					Arguments = "-3.11 \"" + scriptPath + "\"",
					UseShellExecute = true,
					WorkingDirectory = AppDomain.CurrentDomain.BaseDirectory
				};

				_pythonProcess = Process.Start(psi);
				Log("Python hand tracker started");
			}
			catch (Exception ex)
			{
				Log("Failed to start Python: " + ex.Message);
			}
		}

		private void StopTracking()
		{
			_isTrackingRunning = false;

			if (_trackingCts != null)
			{
				_trackingCts.Cancel();
				_trackingCts = null;
			}

			if (_udpClient != null)
			{
				try { _udpClient.Close(); } catch { }
				_udpClient = null;
			}

			if (_pythonProcess != null && !_pythonProcess.HasExited)
			{
				try { _pythonProcess.Kill(); } catch { }
				_pythonProcess = null;
			}

			_handDetected = false;
			btnStartTracking.Enabled = true;
			btnStopTracking.Enabled = false;

			Log("Tracking stopped");
		}

		private void UdpReceiveLoop(CancellationToken token)
		{
			IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);

			while (_isTrackingRunning && !token.IsCancellationRequested)
			{
				try
				{
					if (_udpClient == null || _udpClient.Client == null) break;

					if (_udpClient.Available > 0)
					{
						byte[] data = _udpClient.Receive(ref remoteEP);
						string json = Encoding.UTF8.GetString(data);

						ProcessTrackingData(json);
					}
					else
					{
						Thread.Sleep(5);
					}
				}
				catch (SocketException)
				{
					Thread.Sleep(10);
				}
				catch (ObjectDisposedException)
				{
					break;
				}
				catch (Exception ex)
				{
					System.Diagnostics.Debug.WriteLine($"UDP receive error: {ex.Message}");
					Thread.Sleep(10);
				}
			}
		}

		private void ProcessTrackingData(string json)
		{
			try
			{
				JObject data = JObject.Parse(json);

				_handDetected = data["detected"]?.Value<bool>() ?? false;
				_lastDataReceived = DateTime.Now;

				if (_handDetected)
				{
					JArray anglesArray = data["angles"] as JArray;
					if (anglesArray != null)
					{
						short[] newAngles = new short[6];
						for (int i = 0; i < 6 && i < anglesArray.Count; i++)
						{
							newAngles[i] = ClampShort(anglesArray[i].Value<int>(), 0, 1000);
						}

						for (int i = 0; i < 6; i++)
						{
							_trackedAngles[i] = newAngles[i];
						}
					}

					JArray landmarksArray = data["landmarks"] as JArray;
					if (landmarksArray != null && landmarksArray.Count >= 21)
					{
						float[][] newLandmarks = new float[landmarksArray.Count][];
						for (int i = 0; i < landmarksArray.Count; i++)
						{
							JObject lm = landmarksArray[i] as JObject;
							if (lm != null)
							{
								newLandmarks[i] = new float[]
								{
									lm["x"]?.Value<float>() ?? 0,
									lm["y"]?.Value<float>() ?? 0,
									lm["z"]?.Value<float>() ?? 0
								};
							}
						}
						_handLandmarks = newLandmarks;
					}

					if (chkMirrorControl.Checked)
					{
						short[] commandCopy = new short[6];
						Array.Copy(_trackedAngles, commandCopy, 6);
						_angleCommandQueue.Enqueue(commandCopy);

						try
						{
							this.BeginInvoke(new Action(() =>
							{
								_isUpdatingSliders = true;
								for (int i = 0; i < 6; i++)
								{
									fingerSliders[i].Value = _trackedAngles[i];
									fingerLabels[i].Text = _trackedAngles[i].ToString();
									_targetAngles[i] = _trackedAngles[i];
								}
								_isUpdatingSliders = false;
							}));
						}
						catch { }
					}
				}
			}
			catch (Exception ex)
			{
				System.Diagnostics.Debug.WriteLine($"JSON parse error: {ex.Message}");
			}
		}

		// ============== TRACKING PANEL ==============
		private void PanelTracking_Paint(object sender, PaintEventArgs e)
		{
			Graphics g = e.Graphics;
			g.SmoothingMode = SmoothingMode.AntiAlias;

			int w = panelTracking.Width;
			int h = panelTracking.Height;

			using (Font font = new Font("Segoe UI", 10, FontStyle.Bold))
			using (Font smallFont = new Font("Segoe UI", 9))
			{
				if (!_isTrackingRunning)
				{
					g.DrawString("Click 'Start Tracking' to begin", font, Brushes.White, 10, 10);
					g.DrawString("A Python window will open with camera", smallFont, Brushes.Gray, 10, 35);
					return;
				}

				bool dataFresh = (DateTime.Now - _lastDataReceived).TotalSeconds < 2;

				if (!_handDetected || !dataFresh)
				{
					g.DrawString("Waiting for hand data...", font, Brushes.Yellow, 10, 10);
					g.DrawString("Make sure Python window is open", smallFont, Brushes.Gray, 10, 35);
					return;
				}

				if (_handLandmarks != null && _handLandmarks.Length >= 21)
					DrawHandSkeleton(g, w, h);

				g.DrawString("Hand Detected", font, Brushes.LightGreen, 10, 10);

				int y = 35;
				for (int i = 0; i < 6; i++)
				{
					int barWidth = (int)((_trackedAngles[i] / 1000f) * 100);
					Color barColor = _trackedAngles[i] > 500 ? Color.FromArgb(100, 200, 100) : Color.FromArgb(200, 150, 50);
					g.FillRectangle(new SolidBrush(Color.FromArgb(50, 50, 60)), 10, y, 100, 14);
					g.FillRectangle(new SolidBrush(barColor), 10, y, barWidth, 14);
					g.DrawString(FINGER_NAMES[i] + ": " + _trackedAngles[i], smallFont, Brushes.White, 115, y - 2);
					y += 18;
				}

				if (chkMirrorControl.Checked)
					g.DrawString("✓ Sending to robot", smallFont, Brushes.Cyan, 10, y + 5);
			}
		}

		private void DrawHandSkeleton(Graphics g, int w, int h)
		{
			if (_handLandmarks == null) return;

			int[][] connections = new int[][]
			{
				new[] {0,1}, new[] {1,2}, new[] {2,3}, new[] {3,4},
				new[] {0,5}, new[] {5,6}, new[] {6,7}, new[] {7,8},
				new[] {0,9}, new[] {9,10}, new[] {10,11}, new[] {11,12},
				new[] {0,13}, new[] {13,14}, new[] {14,15}, new[] {15,16},
				new[] {0,17}, new[] {17,18}, new[] {18,19}, new[] {19,20},
				new[] {5,9}, new[] {9,13}, new[] {13,17}
			};

			float minX = float.MaxValue, maxX = float.MinValue;
			float minY = float.MaxValue, maxY = float.MinValue;

			foreach (var lm in _handLandmarks)
			{
				if (lm != null)
				{
					minX = Math.Min(minX, lm[0]);
					maxX = Math.Max(maxX, lm[0]);
					minY = Math.Min(minY, lm[1]);
					maxY = Math.Max(maxY, lm[1]);
				}
			}

			float rangeX = Math.Max(0.001f, maxX - minX);
			float rangeY = Math.Max(0.001f, maxY - minY);
			float scale = Math.Min((w - 140) / rangeX, (h - 40) / rangeY);

			float offsetX = (w - rangeX * scale) / 2 + 60;
			float offsetY = (h - rangeY * scale) / 2;

			using (Pen pen = new Pen(Color.White, 2))
			{
				foreach (int[] conn in connections)
				{
					if (conn[0] < _handLandmarks.Length && conn[1] < _handLandmarks.Length &&
						_handLandmarks[conn[0]] != null && _handLandmarks[conn[1]] != null)
					{
						float x1 = offsetX + (_handLandmarks[conn[0]][0] - minX) * scale;
						float y1 = offsetY + (_handLandmarks[conn[0]][1] - minY) * scale;
						float x2 = offsetX + (_handLandmarks[conn[1]][0] - minX) * scale;
						float y2 = offsetY + (_handLandmarks[conn[1]][1] - minY) * scale;
						g.DrawLine(pen, x1, y1, x2, y2);
					}
				}
			}

			using (SolidBrush brush = new SolidBrush(Color.LightGreen))
			{
				for (int i = 0; i < _handLandmarks.Length; i++)
				{
					if (_handLandmarks[i] != null)
					{
						float x = offsetX + (_handLandmarks[i][0] - minX) * scale;
						float y = offsetY + (_handLandmarks[i][1] - minY) * scale;
						g.FillEllipse(brush, x - 4, y - 4, 8, 8);
					}
				}
			}
		}

		// ============== TIMER ==============
		private void UpdateTimer_Tick(object sender, EventArgs e)
		{
			panelTracking.Invalidate();
		}

		// ============== CLEANUP ==============
		protected override void OnFormClosing(FormClosingEventArgs e)
		{
			_commandProcessTimer?.Stop();
			_commandProcessTimer?.Dispose();
			StopTactileReading();
			StopTracking();
			Disconnect();
			if (updateTimer != null)
			{
				updateTimer.Stop();
				updateTimer.Dispose();
			}
			base.OnFormClosing(e);
		}
	}
}
