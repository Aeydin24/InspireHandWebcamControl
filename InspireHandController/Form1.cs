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
	#region Helper Classes

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
				return Color.FromArgb(225, 225, 225);
			else if (normalized < 0.33f)
			{
				float t = normalized / 0.33f;
				return Color.FromArgb((int)(225 - 225 * t), (int)(225 + 30 * t), (int)(225 - 225 * t));
			}
			else if (normalized < 0.66f)
			{
				float t = (normalized - 0.33f) / 0.33f;
				return Color.FromArgb((int)(255 * t), 255, 0);
			}
			else
			{
				float t = (normalized - 0.66f) / 0.34f;
				return Color.FromArgb(255, (int)(255 - 255 * t), 0);
			}
		}
	}

	#endregion

	#region 3D Types

	public struct Point3D
	{
		public float X, Y, Z;

		public Point3D(float x, float y, float z) { X = x; Y = y; Z = z; }

		public static Point3D operator +(Point3D a, Point3D b) => new Point3D(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
		public static Point3D operator -(Point3D a, Point3D b) => new Point3D(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
		public static Point3D operator *(Point3D p, float s) => new Point3D(p.X * s, p.Y * s, p.Z * s);
		public static Point3D operator /(Point3D p, float s) => new Point3D(p.X / s, p.Y / s, p.Z / s);

		public float Length => (float)Math.Sqrt(X * X + Y * Y + Z * Z);

		public Point3D Normalized
		{
			get
			{
				float len = Length;
				if (len < 0.0001f) return new Point3D(0, 0, 0);
				return new Point3D(X / len, Y / len, Z / len);
			}
		}

		public static float Distance(Point3D a, Point3D b) => (a - b).Length;
		public static Point3D Cross(Point3D a, Point3D b) => new Point3D(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
		public static float Dot(Point3D a, Point3D b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
		public static Point3D Lerp(Point3D a, Point3D b, float t) => new Point3D(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t);
	}

	public class ContactPoint3D
	{
		public Point3D Position;
		public float Pressure;
		public Point3D Normal;
		public string SourceFinger;
		public string SourceRegion;

		public ContactPoint3D Clone() => new ContactPoint3D
		{
			Position = Position,
			Pressure = Pressure,
			Normal = Normal,
			SourceFinger = SourceFinger,
			SourceRegion = SourceRegion
		};
	}

	public enum RecognizedShape
	{
		Unknown,
		Sphere,
		Cylinder,
		Box,
		Flat,
		Rod,
		Irregular
	}

	public class ShapeAnalysisResult
	{
		public RecognizedShape Shape = RecognizedShape.Unknown;
		public float Confidence = 0f;
		public Point3D Center;
		public Point3D Size;
		public float EstimatedRadius;
		public string Description = "";
		public Dictionary<string, float> Metrics = new Dictionary<string, float>();
	}

	#endregion

	#region Hand Kinematics

	public static class HandKinematics
	{
		private const float PROXIMAL_LENGTH = 28f;
		private const float MIDDLE_LENGTH = 18f;
		private const float DISTAL_LENGTH = 12f;
		private const float THUMB_PROXIMAL = 22f;
		private const float THUMB_DISTAL = 16f;

		private static readonly Point3D[] FINGER_BASES = new Point3D[]
		{
			new Point3D(-35, 0, 0),
			new Point3D(-17, 8, 0),
			new Point3D(0, 12, 0),
			new Point3D(17, 8, 0),
			new Point3D(32, -15, -8),
		};

		public static float AngleToBendFactor(int angleValue) => 1.0f - (angleValue / 1000f);

		public static Point3D ComputeFingertipPosition(short[] angles, int fingerIndex)
		{
			if (fingerIndex < 0 || fingerIndex > 4) return new Point3D(0, 0, 0);

			Point3D basePos = FINGER_BASES[fingerIndex];

			if (fingerIndex < 4)
			{
				float bendFactor = AngleToBendFactor(angles[fingerIndex]);
				float maxBendPerJoint = (float)(Math.PI * 0.45);
				float jointBend = bendFactor * maxBendPerJoint;

				float currentAngle = 0;
				float y = basePos.Y;
				float z = basePos.Z;

				currentAngle += jointBend;
				y += PROXIMAL_LENGTH * (float)Math.Cos(currentAngle);
				z += PROXIMAL_LENGTH * (float)Math.Sin(currentAngle);

				currentAngle += jointBend * 1.1f;
				y += MIDDLE_LENGTH * (float)Math.Cos(currentAngle);
				z += MIDDLE_LENGTH * (float)Math.Sin(currentAngle);

				currentAngle += jointBend * 0.7f;
				y += DISTAL_LENGTH * (float)Math.Cos(currentAngle);
				z += DISTAL_LENGTH * (float)Math.Sin(currentAngle);

				return new Point3D(basePos.X, y, z);
			}
			else
			{
				float bendFactor = AngleToBendFactor(angles[4]);
				float rotationFactor = (670 - angles[5]) / 1000f;

				float rotAngle = (float)(Math.PI * 0.3 + rotationFactor * Math.PI * 0.5);
				float bendAngle = bendFactor * (float)(Math.PI * 0.5);

				float cosRot = (float)Math.Cos(rotAngle);
				float sinRot = (float)Math.Sin(rotAngle);
				float cosBend = (float)Math.Cos(bendAngle);
				float sinBend = (float)Math.Sin(bendAngle);

				float px = THUMB_PROXIMAL * cosRot * cosBend;
				float py = THUMB_PROXIMAL * sinBend;
				float pz = THUMB_PROXIMAL * sinRot * cosBend;

				float totalBend = bendAngle * 1.5f;
				float dx = THUMB_DISTAL * cosRot * (float)Math.Cos(totalBend);
				float dy = THUMB_DISTAL * (float)Math.Sin(totalBend);
				float dz = THUMB_DISTAL * sinRot * (float)Math.Cos(totalBend);

				return new Point3D(basePos.X + px + dx, basePos.Y + py + dy, basePos.Z + pz + dz);
			}
		}

		public static Point3D ComputeFingerPadPosition(short[] angles, int fingerIndex)
		{
			Point3D tipPos = ComputeFingertipPosition(angles, fingerIndex);
			Point3D basePos = FINGER_BASES[Math.Min(fingerIndex, 4)];
			return Point3D.Lerp(basePos, tipPos, 0.55f);
		}

		public static Point3D GetPalmCenter() => new Point3D(-5, -8, 8);

		public static Point3D GetFingerDirection(short[] angles, int fingerIndex)
		{
			Point3D tip = ComputeFingertipPosition(angles, fingerIndex);
			Point3D pad = ComputeFingerPadPosition(angles, fingerIndex);
			return (tip - pad).Normalized;
		}
	}

	#endregion

	#region Tactile Shape Analyzer

	public static class TactileShapeAnalyzer
	{
		public class SensorAnalysisResult
		{
			public bool HasContact;
			public float TotalPressure;
			public float PeakPressure;
			public PointF PressureCenter;
			public float CurvatureScore;
			public float PressureSpread;
			public float Circularity;
		}

		public static SensorAnalysisResult AnalyzeTactileGrid(int[] data, int rows, int cols, int threshold = 80)
		{
			var result = new SensorAnalysisResult();

			if (data == null || data.Length != rows * cols)
			{
				result.HasContact = false;
				return result;
			}

			List<(int row, int col, float pressure)> activeCells = new List<(int, int, float)>();
			float maxPressure = 0;
			float totalPressure = 0;

			for (int r = 0; r < rows; r++)
			{
				for (int c = 0; c < cols; c++)
				{
					int value = data[r * cols + c];
					if (value > threshold)
					{
						float pressure = Math.Min(1f, (value - threshold) / 3000f);
						activeCells.Add((r, c, pressure));
						totalPressure += pressure;
						maxPressure = Math.Max(maxPressure, pressure);
					}
				}
			}

			result.TotalPressure = totalPressure;
			result.PeakPressure = maxPressure;
			result.HasContact = activeCells.Count >= 2 && totalPressure > 0.05f;

			if (!result.HasContact) return result;

			float centerR = 0, centerC = 0;
			foreach (var cell in activeCells)
			{
				centerR += cell.row * cell.pressure;
				centerC += cell.col * cell.pressure;
			}
			centerR /= totalPressure;
			centerC /= totalPressure;

			result.PressureCenter = new PointF(centerC / (cols - 1), centerR / (rows - 1));
			result.PressureSpread = (float)activeCells.Count / (rows * cols);
			result.CurvatureScore = CalculateCurvatureFromGradient(data, rows, cols, threshold, centerR, centerC, activeCells);
			result.Circularity = CalculateCircularity(activeCells, centerR, centerC);

			return result;
		}

		private static float CalculateCurvatureFromGradient(int[] data, int rows, int cols,
			int threshold, float centerR, float centerC, List<(int row, int col, float pressure)> activeCells)
		{
			if (activeCells.Count < 4) return 0;

			// Calculate average pressure at different distances from center
			var ringData = new List<(float distance, float pressure)>();

			foreach (var cell in activeCells)
			{
				float dist = (float)Math.Sqrt((cell.row - centerR) * (cell.row - centerR) +
											   (cell.col - centerC) * (cell.col - centerC));
				ringData.Add((dist, cell.pressure));
			}

			if (ringData.Count < 3) return 0;

			// Sort by distance and group into rings
			ringData.Sort((a, b) => a.distance.CompareTo(b.distance));

			float maxDist = ringData.Max(r => r.distance);
			if (maxDist < 0.5f) return 0;

			// Calculate pressure at near vs far distances
			float nearPressure = 0, nearCount = 0;
			float farPressure = 0, farCount = 0;
			float midDist = maxDist * 0.5f;

			foreach (var rd in ringData)
			{
				if (rd.distance < midDist)
				{
					nearPressure += rd.pressure;
					nearCount++;
				}
				else
				{
					farPressure += rd.pressure;
					farCount++;
				}
			}

			if (nearCount > 0) nearPressure /= nearCount;
			if (farCount > 0) farPressure /= farCount;

			// Positive curvature = pressure higher near center (convex/sphere)
			// Negative curvature = pressure higher at edges (concave)
			float curvature = nearPressure - farPressure;

			// Normalize to -1 to 1 range
			curvature = Math.Max(-1f, Math.Min(1f, curvature * 3f));

			return curvature;
		}

		private static float CalculateCircularity(List<(int row, int col, float pressure)> cells, float centerR, float centerC)
		{
			if (cells.Count < 3) return 0;

			float totalWeight = cells.Sum(c => c.pressure);
			float meanDist = 0;

			foreach (var cell in cells)
			{
				float dist = (float)Math.Sqrt((cell.row - centerR) * (cell.row - centerR) +
											   (cell.col - centerC) * (cell.col - centerC));
				meanDist += dist * cell.pressure;
			}
			meanDist /= totalWeight;

			if (meanDist < 0.1f) return 1f;

			float variance = 0;
			foreach (var cell in cells)
			{
				float dist = (float)Math.Sqrt((cell.row - centerR) * (cell.row - centerR) +
											   (cell.col - centerC) * (cell.col - centerC));
				float diff = dist - meanDist;
				variance += diff * diff * cell.pressure;
			}
			variance /= totalWeight;

			float cv = (float)Math.Sqrt(variance) / meanDist;
			return Math.Max(0, 1f - cv);
		}

		public static ShapeAnalysisResult AnalyzeAllSensors(
			int[] littleTip, int[] littlePad,
			int[] ringTip, int[] ringPad,
			int[] middleTip, int[] middlePad,
			int[] indexTip, int[] indexPad,
			int[] thumbTip, int[] thumbPad,
			int[] palm,
			short[] fingerAngles)
		{
			var result = new ShapeAnalysisResult();

			// Analyze each sensor
			var palmAnalysis = AnalyzeTactileGrid(palm, 8, 14);
			var littleTipAnalysis = AnalyzeTactileGrid(littleTip, 3, 3);
			var littlePadAnalysis = AnalyzeTactileGrid(littlePad, 10, 8);
			var ringTipAnalysis = AnalyzeTactileGrid(ringTip, 3, 3);
			var ringPadAnalysis = AnalyzeTactileGrid(ringPad, 10, 8);
			var middleTipAnalysis = AnalyzeTactileGrid(middleTip, 3, 3);
			var middlePadAnalysis = AnalyzeTactileGrid(middlePad, 10, 8);
			var indexTipAnalysis = AnalyzeTactileGrid(indexTip, 3, 3);
			var indexPadAnalysis = AnalyzeTactileGrid(indexPad, 10, 8);
			var thumbTipAnalysis = AnalyzeTactileGrid(thumbTip, 3, 3);
			var thumbPadAnalysis = AnalyzeTactileGrid(thumbPad, 12, 8);

			// Count contacts and calculate metrics
			int activeSensorCount = 0;
			float weightedCurvature = 0;
			float totalWeight = 0;
			int fingerContactCount = 0;
			int tipContactCount = 0;
			int padContactCount = 0;

			var allAnalyses = new (string name, SensorAnalysisResult analysis, float weight, bool isTip)[]
			{
				("Palm", palmAnalysis, 3.0f, false),
				("LittleTip", littleTipAnalysis, 0.5f, true),
				("LittlePad", littlePadAnalysis, 1.0f, false),
				("RingTip", ringTipAnalysis, 0.5f, true),
				("RingPad", ringPadAnalysis, 1.0f, false),
				("MiddleTip", middleTipAnalysis, 0.5f, true),
				("MiddlePad", middlePadAnalysis, 1.0f, false),
				("IndexTip", indexTipAnalysis, 0.5f, true),
				("IndexPad", indexPadAnalysis, 1.0f, false),
				("ThumbTip", thumbTipAnalysis, 0.5f, true),
				("ThumbPad", thumbPadAnalysis, 1.0f, false),
			};

			foreach (var (name, analysis, weight, isTip) in allAnalyses)
			{
				if (analysis.HasContact)
				{
					activeSensorCount++;
					float effectiveWeight = weight * (0.5f + analysis.TotalPressure);
					weightedCurvature += analysis.CurvatureScore * effectiveWeight;
					totalWeight += effectiveWeight;

					if (name != "Palm")
					{
						fingerContactCount++;
						if (isTip) tipContactCount++;
						else padContactCount++;
					}
				}
			}

			if (totalWeight > 0) weightedCurvature /= totalWeight;

			// Calculate finger closure
			float avgFingerClosure = 0;
			for (int i = 0; i < 4; i++)
			{
				avgFingerClosure += (1000 - fingerAngles[i]) / 1000f;
			}
			avgFingerClosure /= 4;

			float thumbClosure = (1000 - fingerAngles[4]) / 1000f;

			// Store metrics
			result.Metrics["ActiveSensors"] = activeSensorCount;
			result.Metrics["WeightedCurvature"] = weightedCurvature;
			result.Metrics["PalmCurvature"] = palmAnalysis.CurvatureScore;
			result.Metrics["PalmCircularity"] = palmAnalysis.Circularity;
			result.Metrics["PalmSpread"] = palmAnalysis.PressureSpread;
			result.Metrics["PalmPressure"] = palmAnalysis.TotalPressure;
			result.Metrics["FingerContacts"] = fingerContactCount;
			result.Metrics["TipContacts"] = tipContactCount;
			result.Metrics["PadContacts"] = padContactCount;
			result.Metrics["FingerClosure"] = avgFingerClosure;
			result.Metrics["ThumbClosure"] = thumbClosure;

			// Classification
			if (activeSensorCount < 2)
			{
				result.Shape = RecognizedShape.Unknown;
				result.Confidence = 0;
				result.Description = "Insufficient contact";
				return result;
			}

			bool hasPalmContact = palmAnalysis.HasContact;
			bool hasSignificantPalmPressure = palmAnalysis.TotalPressure > 0.3f;
			float palmCurvature = palmAnalysis.CurvatureScore;
			float palmCircularity = palmAnalysis.Circularity;
			float palmSpread = palmAnalysis.PressureSpread;

			// Debug output
			System.Diagnostics.Debug.WriteLine($"Palm: contact={hasPalmContact}, pressure={palmAnalysis.TotalPressure:F2}, " +
				$"curvature={palmCurvature:F2}, circularity={palmCircularity:F2}, spread={palmSpread:F2}");
			System.Diagnostics.Debug.WriteLine($"Fingers: contacts={fingerContactCount}, tips={tipContactCount}, " +
				$"pads={padContactCount}, closure={avgFingerClosure:F2}");

			// SPHERE: Palm has convex curvature (high pressure center), circular pattern, fingers curved
			if (hasPalmContact && palmCurvature > 0.15f && fingerContactCount >= 1 && avgFingerClosure > 0.25f)
			{
				float sphereConf = 0.3f;
				sphereConf += Math.Max(0, palmCurvature) * 0.3f;
				sphereConf += palmCircularity * 0.2f;
				sphereConf += Math.Min(avgFingerClosure, 0.8f) * 0.2f;

				result.Shape = RecognizedShape.Sphere;
				result.Confidence = Math.Min(0.95f, sphereConf);
				result.EstimatedRadius = 20 + palmSpread * 50 + (1 - avgFingerClosure) * 30;
				result.Description = $"Sphere (curv={palmCurvature:F2}, circ={palmCircularity:F2})";
				return result;
			}

			// CYLINDER: Palm has curvature but elongated pattern, fingers wrapped
			if (hasPalmContact && palmCurvature > 0.1f && palmCircularity < 0.5f && fingerContactCount >= 2)
			{
				float cylConf = 0.4f;
				cylConf += Math.Max(0, palmCurvature) * 0.2f;
				cylConf += (1 - palmCircularity) * 0.2f;
				cylConf += Math.Min(fingerContactCount / 4f, 0.2f);

				result.Shape = RecognizedShape.Cylinder;
				result.Confidence = Math.Min(0.9f, cylConf);
				result.Description = $"Cylinder (curv={palmCurvature:F2}, circ={palmCircularity:F2})";
				return result;
			}

			// FLAT: Palm has no curvature (uniform pressure), wide spread
			if (hasPalmContact && Math.Abs(palmCurvature) < 0.12f && palmSpread > 0.1f)
			{
				float flatConf = 0.4f;
				flatConf += (1 - Math.Abs(palmCurvature) * 5) * 0.3f;
				flatConf += palmSpread * 0.3f;

				result.Shape = RecognizedShape.Flat;
				result.Confidence = Math.Min(0.9f, flatConf);
				result.Description = $"Flat (curv={palmCurvature:F2}, spread={palmSpread:F2})";
				return result;
			}

			// ROD: Mostly fingertip contact, little palm contact, fingers very curved
			if (tipContactCount >= 2 && !hasSignificantPalmPressure && avgFingerClosure > 0.5f)
			{
				result.Shape = RecognizedShape.Rod;
				result.Confidence = 0.4f + tipContactCount * 0.1f + avgFingerClosure * 0.2f;
				result.Description = $"Rod ({tipContactCount} tips, closure={avgFingerClosure:F2})";
				return result;
			}

			// BOX: Multiple distinct contacts, moderate finger closure
			if (fingerContactCount >= 3 && avgFingerClosure > 0.2f && avgFingerClosure < 0.7f)
			{
				result.Shape = RecognizedShape.Box;
				result.Confidence = 0.4f + fingerContactCount * 0.08f;
				result.Description = $"Box ({fingerContactCount} contacts)";
				return result;
			}

			// IRREGULAR: Default
			result.Shape = RecognizedShape.Irregular;
			result.Confidence = 0.3f;
			result.Description = $"Irregular ({activeSensorCount} sensors)";
			return result;
		}
	}

	#endregion

	#region 3D Visualization Panel

	public class Object3DVisualizationPanel : Panel
	{
		private List<ContactPoint3D> _contactPoints = new List<ContactPoint3D>();
		private ShapeAnalysisResult _currentShape = new ShapeAnalysisResult();
		private short[] _currentAngles = new short[6];
		private readonly object _lock = new object();

		private float _rotationY = 0.4f;
		private float _rotationX = 0.25f;
		private float _zoom = 1.0f;
		private bool _isDragging = false;
		private Point _lastMouse;
		private float _autoRotation = 0f;
		private bool _autoRotate = false;
		private bool _isLeftHand = false;

		public bool IsLeftHand
		{
			get => _isLeftHand;
			set { _isLeftHand = value; Invalidate(); }
		}

		public Object3DVisualizationPanel()
		{
			this.DoubleBuffered = true;
			this.BackColor = Color.FromArgb(18, 20, 25);
			this.Paint += OnPaint;
			this.MouseDown += OnMouseDown;
			this.MouseMove += OnMouseMove;
			this.MouseUp += OnMouseUp;
			this.MouseWheel += OnMouseWheel;

			for (int i = 0; i < 6; i++)
				_currentAngles[i] = 1000;
		}

		public void UpdateData(
			short[] angles,
			int[] littleTip, int[] littlePad,
			int[] ringTip, int[] ringPad,
			int[] middleTip, int[] middlePad,
			int[] indexTip, int[] indexPad,
			int[] thumbTip, int[] thumbPad,
			int[] palm)
		{
			lock (_lock)
			{
				if (angles != null && angles.Length >= 6)
					Array.Copy(angles, _currentAngles, 6);

				_contactPoints.Clear();

				ProcessFingerTactile(littleTip, 0, "Little", "tip", 3, 3);
				ProcessFingerTactile(littlePad, 0, "Little", "pad", 10, 8);
				ProcessFingerTactile(ringTip, 1, "Ring", "tip", 3, 3);
				ProcessFingerTactile(ringPad, 1, "Ring", "pad", 10, 8);
				ProcessFingerTactile(middleTip, 2, "Middle", "tip", 3, 3);
				ProcessFingerTactile(middlePad, 2, "Middle", "pad", 10, 8);
				ProcessFingerTactile(indexTip, 3, "Index", "tip", 3, 3);
				ProcessFingerTactile(indexPad, 3, "Index", "pad", 10, 8);
				ProcessFingerTactile(thumbTip, 4, "Thumb", "tip", 3, 3);
				ProcessFingerTactile(thumbPad, 4, "Thumb", "pad", 12, 8);
				ProcessPalmTactile(palm, 8, 14);

				// Use tactile-based shape analyzer
				_currentShape = TactileShapeAnalyzer.AnalyzeAllSensors(
					littleTip, littlePad,
					ringTip, ringPad,
					middleTip, middlePad,
					indexTip, indexPad,
					thumbTip, thumbPad,
					palm,
					_currentAngles
				);

				if (_contactPoints.Count >= 3)
				{
					_currentShape.Center = CalculateCentroid(_contactPoints);
					_currentShape.Size = CalculateBoundingBox(_contactPoints);
					if (_currentShape.EstimatedRadius < 1)
						_currentShape.EstimatedRadius = _currentShape.Size.Length / 2;
				}
			}

			if (_autoRotate) _autoRotation += 0.02f;
			Invalidate();
		}

		private Point3D CalculateCentroid(List<ContactPoint3D> contacts)
		{
			if (contacts.Count == 0) return new Point3D(0, 0, 0);
			float totalWeight = contacts.Sum(c => c.Pressure);
			if (totalWeight < 0.001f) totalWeight = contacts.Count;
			return new Point3D(
				contacts.Sum(c => c.Position.X * c.Pressure) / totalWeight,
				contacts.Sum(c => c.Position.Y * c.Pressure) / totalWeight,
				contacts.Sum(c => c.Position.Z * c.Pressure) / totalWeight
			);
		}

		private Point3D CalculateBoundingBox(List<ContactPoint3D> contacts)
		{
			if (contacts.Count == 0) return new Point3D(0, 0, 0);
			return new Point3D(
				contacts.Max(c => c.Position.X) - contacts.Min(c => c.Position.X),
				contacts.Max(c => c.Position.Y) - contacts.Min(c => c.Position.Y),
				contacts.Max(c => c.Position.Z) - contacts.Min(c => c.Position.Z)
			);
		}

		private void ProcessFingerTactile(int[] data, int fingerIndex, string fingerName, string region, int rows, int cols)
		{
			if (data == null || data.Length != rows * cols) return;

			const int THRESHOLD = 80;

			Point3D basePos = region == "tip"
				? GetAdjustedFingertipPosition(_currentAngles, fingerIndex)
				: GetAdjustedFingerPadPosition(_currentAngles, fingerIndex);

			Point3D fingerDir = HandKinematics.GetFingerDirection(_currentAngles, fingerIndex);
			if (_isLeftHand)
				fingerDir = new Point3D(-fingerDir.X, fingerDir.Y, fingerDir.Z);

			Point3D perpX = new Point3D(1, 0, 0);
			if (Math.Abs(Point3D.Dot(fingerDir, perpX)) > 0.9f)
				perpX = new Point3D(0, 1, 0);
			Point3D perpZ = Point3D.Cross(fingerDir, perpX).Normalized;
			perpX = Point3D.Cross(perpZ, fingerDir).Normalized;

			float spreadX = region == "tip" ? 6f : 10f;
			float spreadZ = region == "tip" ? 6f : 12f;

			for (int r = 0; r < rows; r++)
			{
				for (int c = 0; c < cols; c++)
				{
					int value = data[r * cols + c];
					if (value > THRESHOLD)
					{
						float pressure = Math.Min(1f, (value - THRESHOLD) / 2500f);
						float offsetX = (c - cols / 2f) / cols * spreadX;
						float offsetZ = (r - rows / 2f) / rows * spreadZ;

						Point3D contactPos = basePos + perpX * offsetX + perpZ * offsetZ;
						Point3D palmCenter = GetAdjustedPalmCenter();
						Point3D normal = (palmCenter - contactPos).Normalized;

						_contactPoints.Add(new ContactPoint3D
						{
							Position = contactPos,
							Pressure = pressure,
							SourceFinger = fingerName,
							SourceRegion = region,
							Normal = normal
						});
					}
				}
			}
		}

		private void ProcessPalmTactile(int[] data, int rows, int cols)
		{
			if (data == null || data.Length < rows * cols) return;

			const int THRESHOLD = 80;
			Point3D palmCenter = GetAdjustedPalmCenter();

			for (int r = 0; r < rows; r++)
			{
				for (int c = 0; c < cols; c++)
				{
					int idx = r * cols + c;
					if (idx >= data.Length) continue;

					int value = data[idx];
					if (value > THRESHOLD)
					{
						float pressure = Math.Min(1f, (value - THRESHOLD) / 2500f);
						float offsetX = (c - cols / 2f) / cols * 55f;
						float offsetY = (r - rows / 2f) / rows * 35f;

						_contactPoints.Add(new ContactPoint3D
						{
							Position = new Point3D(palmCenter.X + offsetX, palmCenter.Y + offsetY, palmCenter.Z),
							Pressure = pressure,
							SourceFinger = "Palm",
							SourceRegion = "palm",
							Normal = new Point3D(0, 0, 1)
						});
					}
				}
			}
		}

		public void Clear()
		{
			lock (_lock)
			{
				_contactPoints.Clear();
				_currentShape = new ShapeAnalysisResult { Shape = RecognizedShape.Unknown, Description = "No contact" };
			}
			Invalidate();
		}

		private PointF Project3D(Point3D p, int centerX, int centerY, float scale)
		{
			Point3D point = p;
			if (_isLeftHand)
				point = new Point3D(-p.X, p.Y, p.Z);

			float totalRotY = _rotationY + (_autoRotate ? _autoRotation : 0);
			float cosY = (float)Math.Cos(totalRotY);
			float sinY = (float)Math.Sin(totalRotY);
			float cosX = (float)Math.Cos(_rotationX);
			float sinX = (float)Math.Sin(_rotationX);

			float x1 = point.X * cosY - point.Y * sinY;
			float y1 = point.X * sinY + point.Y * cosY;
			float z1 = point.Z;
			float y2 = y1 * cosX - z1 * sinX;
			float z2 = y1 * sinX + z1 * cosX;

			float perspective = 250f / (250f + y2);
			return new PointF(centerX + x1 * scale * _zoom * perspective, centerY - z2 * scale * _zoom * perspective);
		}

		private float GetDepth(Point3D p)
		{
			Point3D point = p;
			if (_isLeftHand)
				point = new Point3D(-p.X, p.Y, p.Z);

			float totalRotY = _rotationY + (_autoRotate ? _autoRotation : 0);
			float cosY = (float)Math.Cos(totalRotY);
			float sinY = (float)Math.Sin(totalRotY);
			float cosX = (float)Math.Cos(_rotationX);
			float sinX = (float)Math.Sin(_rotationX);
			float y1 = point.X * sinY + point.Y * cosY;
			return y1 * cosX - point.Z * sinX;
		}

		private Point3D GetAdjustedFingertipPosition(short[] angles, int fingerIndex)
		{
			var pos = HandKinematics.ComputeFingertipPosition(angles, fingerIndex);
			if (_isLeftHand)
				pos = new Point3D(-pos.X, pos.Y, pos.Z);
			return pos;
		}

		private Point3D GetAdjustedFingerPadPosition(short[] angles, int fingerIndex)
		{
			var pos = HandKinematics.ComputeFingerPadPosition(angles, fingerIndex);
			if (_isLeftHand)
				pos = new Point3D(-pos.X, pos.Y, pos.Z);
			return pos;
		}

		private Point3D GetAdjustedPalmCenter()
		{
			var pos = HandKinematics.GetPalmCenter();
			if (_isLeftHand)
				pos = new Point3D(-pos.X, pos.Y, pos.Z);
			return pos;
		}

		private Point3D GetAdjustedFingerBase(int fingerIndex)
		{
			var bases = new Point3D[]
			{
				new Point3D(-35, 0, 0),
				new Point3D(-17, 8, 0),
				new Point3D(0, 12, 0),
				new Point3D(17, 8, 0),
				new Point3D(32, -15, -8),
			};

			var pos = bases[fingerIndex];
			if (_isLeftHand)
				pos = new Point3D(-pos.X, pos.Y, pos.Z);
			return pos;
		}

		private void OnMouseDown(object sender, MouseEventArgs e)
		{
			if (e.Button == MouseButtons.Left) { _isDragging = true; _lastMouse = e.Location; _autoRotate = false; }
			else if (e.Button == MouseButtons.Right) { _autoRotate = !_autoRotate; }
		}

		private void OnMouseMove(object sender, MouseEventArgs e)
		{
			if (_isDragging)
			{
				_rotationY += (e.X - _lastMouse.X) * 0.01f;
				_rotationX += (e.Y - _lastMouse.Y) * 0.01f;
				_rotationX = Math.Max(-1.5f, Math.Min(1.5f, _rotationX));
				_lastMouse = e.Location;
				Invalidate();
			}
		}

		private void OnMouseUp(object sender, MouseEventArgs e) { _isDragging = false; }

		private void OnMouseWheel(object sender, MouseEventArgs e)
		{
			_zoom = Math.Max(0.5f, Math.Min(2.5f, _zoom + e.Delta * 0.001f));
			Invalidate();
		}

		private void OnPaint(object sender, PaintEventArgs e)
		{
			Graphics g = e.Graphics;
			g.SmoothingMode = SmoothingMode.AntiAlias;
			g.TextRenderingHint = System.Drawing.Text.TextRenderingHint.ClearTypeGridFit;

			int w = Width, h = Height;
			int centerX = w / 2, centerY = h / 2 + 15;
			float scale = Math.Min(w, h) / 160f;

			using (var bgBrush = new LinearGradientBrush(new Point(0, 0), new Point(0, h),
				Color.FromArgb(18, 20, 25), Color.FromArgb(28, 32, 40)))
			{
				g.FillRectangle(bgBrush, 0, 0, w, h);
			}

			using (var borderPen = new Pen(Color.FromArgb(50, 60, 70), 1))
			{
				g.DrawRectangle(borderPen, 0, 0, w - 1, h - 1);
			}

			List<ContactPoint3D> contacts;
			ShapeAnalysisResult shape;
			short[] angles;

			lock (_lock)
			{
				contacts = _contactPoints.Select(c => c.Clone()).ToList();
				shape = _currentShape;
				angles = (short[])_currentAngles.Clone();
			}

			DrawAxes(g, centerX, centerY, scale);
			DrawHandSkeleton(g, angles, centerX, centerY, scale);

			if (contacts.Count > 0)
				DrawContactPoints(g, contacts, centerX, centerY, scale);

			if (shape.Shape != RecognizedShape.Unknown)
				DrawRecognizedShape(g, shape, contacts, centerX, centerY, scale);

			DrawInfoOverlay(g, contacts.Count, shape, w, h);
			DrawInstructions(g, w, h);
		}

		private void DrawAxes(Graphics g, int cx, int cy, float scale)
		{
			float axisLen = 40f;
			Point3D axisOrigin = new Point3D(75, 50, 30);
			var origin = Project3D(axisOrigin, cx, cy, scale);

			using (var penX = new Pen(Color.FromArgb(255, 220, 80, 80), 2.5f))
			using (var penY = new Pen(Color.FromArgb(255, 80, 220, 80), 2.5f))
			using (var penZ = new Pen(Color.FromArgb(255, 80, 80, 220), 2.5f))
			using (var font = new Font("Segoe UI", 9, FontStyle.Bold))
			{
				var xEnd = Project3D(axisOrigin + new Point3D(axisLen, 0, 0), cx, cy, scale);
				var yEnd = Project3D(axisOrigin + new Point3D(0, axisLen, 0), cx, cy, scale);
				var zEnd = Project3D(axisOrigin + new Point3D(0, 0, axisLen), cx, cy, scale);

				g.DrawLine(penX, origin, xEnd);
				g.DrawLine(penY, origin, yEnd);
				g.DrawLine(penZ, origin, zEnd);

				g.DrawString("X", font, new SolidBrush(Color.FromArgb(255, 220, 80, 80)), xEnd.X + 5, xEnd.Y - 10);
				g.DrawString("Y", font, new SolidBrush(Color.FromArgb(255, 80, 220, 80)), yEnd.X - 15, yEnd.Y - 10);
				g.DrawString("Z", font, new SolidBrush(Color.FromArgb(255, 80, 80, 220)), zEnd.X + 5, zEnd.Y - 10);
			}
		}

		private void DrawHandSkeleton(Graphics g, short[] angles, int cx, int cy, float scale)
		{
			using (var bonePen = new Pen(Color.FromArgb(60, 120, 130, 140), 1.5f))
			using (var jointBrush = new SolidBrush(Color.FromArgb(80, 150, 160, 170)))
			{
				Point3D[] palmCorners = { new Point3D(-38, -5, 5), new Point3D(25, -5, 5), new Point3D(30, 15, 5), new Point3D(-38, 10, 5) };
				if (_isLeftHand)
					for (int i = 0; i < palmCorners.Length; i++)
						palmCorners[i] = new Point3D(-palmCorners[i].X, palmCorners[i].Y, palmCorners[i].Z);

				PointF[] palmPoly = palmCorners.Select(p => Project3D(p, cx, cy, scale)).ToArray();
				using (var palmBrush = new SolidBrush(Color.FromArgb(30, 100, 110, 120)))
					g.FillPolygon(palmBrush, palmPoly);

				for (int i = 0; i < 5; i++)
				{
					var baseScreen = Project3D(GetAdjustedFingerBase(i), cx, cy, scale);
					var padScreen = Project3D(GetAdjustedFingerPadPosition(angles, i), cx, cy, scale);
					var tipScreen = Project3D(GetAdjustedFingertipPosition(angles, i), cx, cy, scale);

					g.DrawLine(bonePen, baseScreen, padScreen);
					g.DrawLine(bonePen, padScreen, tipScreen);
					g.FillEllipse(jointBrush, baseScreen.X - 3, baseScreen.Y - 3, 6, 6);
					g.FillEllipse(jointBrush, padScreen.X - 2, padScreen.Y - 2, 4, 4);
					g.FillEllipse(jointBrush, tipScreen.X - 3, tipScreen.Y - 3, 6, 6);
				}
			}
		}

		private void DrawContactPoints(Graphics g, List<ContactPoint3D> contacts, int cx, int cy, float scale)
		{
			foreach (var contact in contacts.OrderBy(c => GetDepth(c.Position)))
			{
				var screen = Project3D(contact.Position, cx, cy, scale);
				float baseRadius = 3 + contact.Pressure * 10;

				for (int i = 3; i >= 0; i--)
				{
					float r = baseRadius + i * 2.5f;
					using (var brush = new SolidBrush(GetPressureColor(contact.Pressure, (int)(contact.Pressure * 80 / (i + 1)))))
						g.FillEllipse(brush, screen.X - r, screen.Y - r, r * 2, r * 2);
				}

				float coreR = baseRadius * 0.6f;
				using (var brush = new SolidBrush(GetPressureColor(contact.Pressure, 230)))
					g.FillEllipse(brush, screen.X - coreR, screen.Y - coreR, coreR * 2, coreR * 2);
			}
		}

		private void DrawRecognizedShape(Graphics g, ShapeAnalysisResult shape, List<ContactPoint3D> contacts, int cx, int cy, float scale)
		{
			Color shapeColor = GetShapeColor(shape.Shape);
			Color outlineColor = Color.FromArgb(Math.Min(255, shapeColor.A * 3), shapeColor.R, shapeColor.G, shapeColor.B);

			switch (shape.Shape)
			{
				case RecognizedShape.Sphere:
					DrawSphere3D(g, shape.Center, shape.EstimatedRadius * 0.8f, shapeColor, outlineColor, cx, cy, scale);
					break;
				case RecognizedShape.Box:
					DrawBox3D(g, shape.Center, shape.Size * 0.9f, shapeColor, outlineColor, cx, cy, scale);
					break;
				case RecognizedShape.Cylinder:
				case RecognizedShape.Rod:
					DrawCylinder3D(g, shape.Center, shape.Size, shapeColor, outlineColor, cx, cy, scale);
					break;
				case RecognizedShape.Flat:
					DrawFlatSurface(g, shape.Center, shape.Size, shapeColor, outlineColor, cx, cy, scale);
					break;
				default:
					DrawConvexHull(g, contacts, shapeColor, outlineColor, cx, cy, scale);
					break;
			}
		}

		private Color GetShapeColor(RecognizedShape shape) => shape switch
		{
			RecognizedShape.Sphere => Color.FromArgb(50, 80, 180, 255),
			RecognizedShape.Cylinder => Color.FromArgb(50, 255, 180, 80),
			RecognizedShape.Rod => Color.FromArgb(50, 255, 140, 60),
			RecognizedShape.Box => Color.FromArgb(50, 200, 100, 220),
			RecognizedShape.Flat => Color.FromArgb(50, 100, 220, 150),
			_ => Color.FromArgb(35, 180, 180, 180)
		};

		private void DrawSphere3D(Graphics g, Point3D center, float radius, Color fill, Color outline, int cx, int cy, float scale)
		{
			var centerScreen = Project3D(center, cx, cy, scale);
			float screenRadius = Math.Max(5, radius * scale * _zoom * 0.7f);

			using (var path = new GraphicsPath())
			{
				path.AddEllipse(centerScreen.X - screenRadius, centerScreen.Y - screenRadius, screenRadius * 2, screenRadius * 2);
				using (var gradBrush = new PathGradientBrush(path))
				{
					gradBrush.CenterColor = Color.FromArgb(100, 255, 255, 255);
					gradBrush.SurroundColors = new[] { fill };
					gradBrush.CenterPoint = new PointF(centerScreen.X - screenRadius * 0.35f, centerScreen.Y - screenRadius * 0.35f);
					g.FillPath(gradBrush, path);
				}
				using (var pen = new Pen(outline, 2)) g.DrawPath(pen, path);
			}

			using (var pen = new Pen(Color.FromArgb(60, outline.R, outline.G, outline.B), 1) { DashStyle = DashStyle.Dash })
				g.DrawEllipse(pen, centerScreen.X - screenRadius, centerScreen.Y - screenRadius * 0.3f, screenRadius * 2, screenRadius * 0.6f);
		}

		private void DrawBox3D(Graphics g, Point3D center, Point3D size, Color fill, Color outline, int cx, int cy, float scale)
		{
			float hx = Math.Max(5, size.X / 2 * 0.8f);
			float hy = Math.Max(5, size.Y / 2 * 0.8f);
			float hz = Math.Max(5, size.Z / 2 * 0.8f);

			Point3D[] corners = {
				new Point3D(center.X - hx, center.Y - hy, center.Z - hz),
				new Point3D(center.X + hx, center.Y - hy, center.Z - hz),
				new Point3D(center.X + hx, center.Y + hy, center.Z - hz),
				new Point3D(center.X - hx, center.Y + hy, center.Z - hz),
				new Point3D(center.X - hx, center.Y - hy, center.Z + hz),
				new Point3D(center.X + hx, center.Y - hy, center.Z + hz),
				new Point3D(center.X + hx, center.Y + hy, center.Z + hz),
				new Point3D(center.X - hx, center.Y + hy, center.Z + hz),
			};

			PointF[] screenCorners = corners.Select(c => Project3D(c, cx, cy, scale)).ToArray();
			int[][] faces = { new[] { 0, 1, 2, 3 }, new[] { 4, 5, 6, 7 }, new[] { 0, 1, 5, 4 }, new[] { 2, 3, 7, 6 }, new[] { 0, 3, 7, 4 }, new[] { 1, 2, 6, 5 } };

			foreach (var face in faces.OrderBy(f => f.Average(i => GetDepth(corners[i]))))
			{
				using (var brush = new SolidBrush(Color.FromArgb((int)(fill.A * 0.8f), fill.R, fill.G, fill.B)))
					g.FillPolygon(brush, face.Select(i => screenCorners[i]).ToArray());
			}

			int[][] edges = { new[] { 0, 1 }, new[] { 1, 2 }, new[] { 2, 3 }, new[] { 3, 0 }, new[] { 4, 5 }, new[] { 5, 6 }, new[] { 6, 7 }, new[] { 7, 4 }, new[] { 0, 4 }, new[] { 1, 5 }, new[] { 2, 6 }, new[] { 3, 7 } };
			using (var pen = new Pen(outline, 1.5f))
				foreach (var edge in edges) g.DrawLine(pen, screenCorners[edge[0]], screenCorners[edge[1]]);
		}

		private void DrawCylinder3D(Graphics g, Point3D center, Point3D size, Color fill, Color outline, int cx, int cy, float scale)
		{
			var centerScreen = Project3D(center, cx, cy, scale);
			float[] dims = { size.X, size.Y, size.Z };
			float height = Math.Max(15, dims.Max() * scale * _zoom * 0.4f);
			float radius = Math.Max(8, (dims.Sum() - dims.Max()) / 2 * scale * _zoom * 0.3f);

			using (var brush = new SolidBrush(fill))
			{
				g.FillEllipse(brush, centerScreen.X - radius, centerScreen.Y - height / 2 - radius * 0.3f, radius * 2, radius * 0.6f);
				g.FillRectangle(brush, centerScreen.X - radius, centerScreen.Y - height / 2, radius * 2, height);
				using (var bottomBrush = new SolidBrush(Color.FromArgb(Math.Min(255, fill.A + 30), fill.R, fill.G, fill.B)))
					g.FillEllipse(bottomBrush, centerScreen.X - radius, centerScreen.Y + height / 2 - radius * 0.3f, radius * 2, radius * 0.6f);
			}

			using (var pen = new Pen(outline, 1.5f))
			{
				g.DrawLine(pen, centerScreen.X - radius, centerScreen.Y - height / 2, centerScreen.X - radius, centerScreen.Y + height / 2);
				g.DrawLine(pen, centerScreen.X + radius, centerScreen.Y - height / 2, centerScreen.X + radius, centerScreen.Y + height / 2);
				g.DrawEllipse(pen, centerScreen.X - radius, centerScreen.Y - height / 2 - radius * 0.3f, radius * 2, radius * 0.6f);
				g.DrawEllipse(pen, centerScreen.X - radius, centerScreen.Y + height / 2 - radius * 0.3f, radius * 2, radius * 0.6f);
			}
		}

		private void DrawFlatSurface(Graphics g, Point3D center, Point3D size, Color fill, Color outline, int cx, int cy, float scale)
		{
			float hx = Math.Max(size.X, 20) / 2;
			float hy = Math.Max(size.Y, 20) / 2;

			Point3D[] corners = {
				new Point3D(center.X - hx, center.Y - hy, center.Z),
				new Point3D(center.X + hx, center.Y - hy, center.Z),
				new Point3D(center.X + hx, center.Y + hy, center.Z),
				new Point3D(center.X - hx, center.Y + hy, center.Z),
			};

			PointF[] screenCorners = corners.Select(c => Project3D(c, cx, cy, scale)).ToArray();

			using (var brush = new SolidBrush(fill)) g.FillPolygon(brush, screenCorners);
			using (var pen = new Pen(outline, 2)) g.DrawPolygon(pen, screenCorners);
		}

		private void DrawConvexHull(Graphics g, List<ContactPoint3D> contacts, Color fill, Color outline, int cx, int cy, float scale)
		{
			if (contacts.Count < 3) return;

			var screenPoints = contacts.Select(c => Project3D(c.Position, cx, cy, scale)).ToList();
			var hull = ComputeConvexHull2D(screenPoints);
			if (hull.Count < 3) return;

			using (var brush = new SolidBrush(fill)) g.FillPolygon(brush, hull.ToArray());
			using (var pen = new Pen(outline, 1.5f) { LineJoin = LineJoin.Round }) g.DrawPolygon(pen, hull.ToArray());
		}

		private List<PointF> ComputeConvexHull2D(List<PointF> points)
		{
			if (points.Count < 3) return points;

			PointF start = points.OrderBy(p => p.Y).ThenBy(p => p.X).First();
			var sorted = points.OrderBy(p =>
			{
				if (Math.Abs(p.X - start.X) < 0.001f && Math.Abs(p.Y - start.Y) < 0.001f) return -4f;
				return (float)Math.Atan2(p.Y - start.Y, p.X - start.X);
			}).ToList();

			Stack<PointF> hull = new Stack<PointF>();
			foreach (var p in sorted)
			{
				while (hull.Count >= 2)
				{
					var top = hull.Pop();
					var next = hull.Peek();
					if ((top.X - next.X) * (p.Y - next.Y) - (top.Y - next.Y) * (p.X - next.X) > 0)
					{
						hull.Push(top);
						break;
					}
				}
				hull.Push(p);
			}
			return hull.ToList();
		}

		private void DrawInfoOverlay(Graphics g, int contactCount, ShapeAnalysisResult shape, int w, int h)
		{
			using (var font = new Font("Segoe UI", 9, FontStyle.Bold))
			using (var smallFont = new Font("Segoe UI", 8))
			using (var tinyFont = new Font("Consolas", 7))
			{
				int y = 8, x = 8;

				string shapeIcon = shape.Shape switch
				{
					RecognizedShape.Sphere => "●",
					RecognizedShape.Cylinder => "▮",
					RecognizedShape.Box => "■",
					RecognizedShape.Flat => "▬",
					RecognizedShape.Rod => "│",
					_ => "○"
				};

				Color shapeTextColor = shape.Shape != RecognizedShape.Unknown
					? Color.FromArgb(220, Math.Min(255, GetShapeColor(shape.Shape).R + 50),
						Math.Min(255, GetShapeColor(shape.Shape).G + 50),
						Math.Min(255, GetShapeColor(shape.Shape).B + 50))
					: Color.Gray;

				g.DrawString($"{shapeIcon} {shape.Shape}", font, new SolidBrush(shapeTextColor), x, y);
				y += 18;

				if (shape.Shape != RecognizedShape.Unknown)
				{
					g.DrawString($"Conf: {shape.Confidence:P0}", smallFont, Brushes.LightGray, x, y);
					g.FillRectangle(new SolidBrush(Color.FromArgb(60, 60, 60)), x + 55, y + 2, 50, 8);
					g.FillRectangle(new SolidBrush(shapeTextColor), x + 55, y + 2, (int)(50 * shape.Confidence), 8);
					y += 16;
				}

				g.DrawString(shape.Description, smallFont, Brushes.DarkGray, x, y);
				y += 16;
				g.DrawString($"Contacts: {contactCount}", smallFont, Brushes.Gray, x, y);

				// Show key metrics
				if (shape.Metrics.Count > 0)
				{
					y = h - 80;
					var keysToShow = new[] { "PalmCurvature", "PalmCircularity", "FingerClosure", "PalmPressure" };
					foreach (var key in keysToShow)
					{
						if (shape.Metrics.TryGetValue(key, out float val))
						{
							g.DrawString($"{key}: {val:F2}", tinyFont, new SolidBrush(Color.FromArgb(120, 120, 120)), x, y);
							y += 12;
						}
					}
				}
			}
		}

		private void DrawInstructions(Graphics g, int w, int h)
		{
			using (var font = new Font("Segoe UI", 7))
			{
				string instructions = "Drag: rotate | Scroll: zoom | RClick: auto";
				var size = g.MeasureString(instructions, font);
				g.DrawString(instructions, font, new SolidBrush(Color.FromArgb(80, 80, 80)), w - size.Width - 5, h - size.Height - 3);
			}
		}

		private Color GetPressureColor(float pressure, int alpha)
		{
			pressure = Math.Max(0, Math.Min(1, pressure));
			int r, gr, b;

			if (pressure < 0.25f) { float t = pressure / 0.25f; r = 50; gr = (int)(100 + t * 100); b = 200; }
			else if (pressure < 0.5f) { float t = (pressure - 0.25f) / 0.25f; r = (int)(50 + t * 50); gr = 200; b = (int)(200 - t * 150); }
			else if (pressure < 0.75f) { float t = (pressure - 0.5f) / 0.25f; r = (int)(100 + t * 155); gr = 200; b = 50; }
			else { float t = (pressure - 0.75f) / 0.25f; r = 255; gr = (int)(200 - t * 150); b = 50; }

			return Color.FromArgb(alpha, r, gr, b);
		}
	}

	#endregion

	#region Main Form

	public partial class Form1 : Form
	{
		private TcpClient _tcpClient;
		private NetworkStream _stream;
		private ushort _transactionId = 0;
		private bool _isConnected = false;
		private readonly object _streamLock = new object();
		private readonly SemaphoreSlim _modbusSemaphore = new SemaphoreSlim(1, 1);

		private string _ipAddress = "192.168.11.210";
		private int _port = 6000;

		private UdpClient _udpClient;
		private Process _pythonProcess;
		private volatile bool _isTrackingRunning = false;
		private CancellationTokenSource _trackingCts;
		private volatile bool _handDetected = false;
		private float[][] _handLandmarks;
		private DateTime _lastDataReceived = DateTime.MinValue;

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
		private const int ANGLE_ACT_ADDRESS = 1546;

		private bool _isReadingTactile = false;
		private CancellationTokenSource _tactileCts;

		private short[] _currentAngles = new short[6];
		private short[] _targetAngles = new short[6];
		private volatile short[] _trackedAngles = new short[6];
		private short[] _actualAnglesFromRobot = new short[6];
		private short _globalSpeed = 500;
		private short _globalForce = 1000;

		private ConcurrentQueue<short[]> _angleCommandQueue = new ConcurrentQueue<short[]>();
		private System.Windows.Forms.Timer _commandProcessTimer;

		private TextBox txtIpAddress;
		private NumericUpDown numPort;
		private Button btnConnect, btnDisconnect;
		private Label lblStatus;
		private TrackBar trackSpeed, trackForce;
		private Label lblSpeed, lblForce;
		private Button btnOpenHand, btnCloseHand, btnStartTracking, btnStopTracking;
		private CheckBox chkMirrorControl;
		private DoubleBufferedPanel panelTracking;
		private Panel panelTactile;
		private Object3DVisualizationPanel panel3DObject;
		private TextBox txtLog;
		private TrackBar[] fingerSliders = new TrackBar[6];
		private Label[] fingerLabels = new Label[6];
		private Label[] actualAngleLabels = new Label[6];

		private TactileSensorGrid gridLittleTip, gridLittleNail, gridLittlePad;
		private TactileSensorGrid gridRingTip, gridRingNail, gridRingPad;
		private TactileSensorGrid gridMiddleTip, gridMiddleNail, gridMiddlePad;
		private TactileSensorGrid gridIndexTip, gridIndexNail, gridIndexPad;
		private TactileSensorGrid gridThumbTip, gridThumbNail, gridThumbMid, gridThumbPad;
		private TactileSensorGrid gridPalm;

		private int[] _lastLittleTip, _lastLittlePad, _lastRingTip, _lastRingPad;
		private int[] _lastMiddleTip, _lastMiddlePad, _lastIndexTip, _lastIndexPad;
		private int[] _lastThumbTip, _lastThumbPad, _lastPalm;

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
				_actualAnglesFromRobot[i] = 1000;
			}
		}

		private static int Clamp(int value, int min, int max) => value < min ? min : (value > max ? max : value);
		private static short ClampShort(int value, int min, int max) => (short)Clamp(value, min, max);

		private void SetupUI()
		{
			this.Text = "Inspire Robotics Dexterous Hand Controller";
			this.Size = new Size(1480, 920);
			this.MinimumSize = new Size(1400, 870);
			this.StartPosition = FormStartPosition.CenterScreen;
			this.BackColor = Color.FromArgb(245, 245, 250);

			// Connection Group
			GroupBox grpConnection = new GroupBox { Text = "Connection", Location = new Point(10, 10), Size = new Size(350, 100) };
			this.Controls.Add(grpConnection);

			grpConnection.Controls.Add(new Label { Text = "IP Address:", Location = new Point(10, 25), AutoSize = true });
			txtIpAddress = new TextBox { Text = _ipAddress, Location = new Point(80, 22), Size = new Size(120, 23) };
			grpConnection.Controls.Add(txtIpAddress);

			grpConnection.Controls.Add(new Label { Text = "Port:", Location = new Point(210, 25), AutoSize = true });
			numPort = new NumericUpDown { Minimum = 1, Maximum = 65535, Value = _port, Location = new Point(250, 22), Size = new Size(80, 23) };
			grpConnection.Controls.Add(numPort);

			btnConnect = new Button { Text = "Connect", Location = new Point(10, 55), Size = new Size(100, 30) };
			btnConnect.Click += BtnConnect_Click;
			grpConnection.Controls.Add(btnConnect);

			btnDisconnect = new Button { Text = "Disconnect", Location = new Point(120, 55), Size = new Size(100, 30), Enabled = false };
			btnDisconnect.Click += BtnDisconnect_Click;
			grpConnection.Controls.Add(btnDisconnect);

			lblStatus = new Label { Text = "● Disconnected", Location = new Point(230, 62), AutoSize = true, ForeColor = Color.Red, Font = new Font("Segoe UI", 9, FontStyle.Bold) };
			grpConnection.Controls.Add(lblStatus);

			// Speed & Force Group
			GroupBox grpControl = new GroupBox { Text = "Global Speed & Force", Location = new Point(370, 10), Size = new Size(350, 100) };
			this.Controls.Add(grpControl);

			grpControl.Controls.Add(new Label { Text = "Speed:", Location = new Point(10, 25), AutoSize = true });
			trackSpeed = new TrackBar { Minimum = 0, Maximum = 1000, Value = 500, Location = new Point(60, 20), Size = new Size(200, 45), TickFrequency = 100 };
			trackSpeed.ValueChanged += TrackSpeed_ValueChanged;
			grpControl.Controls.Add(trackSpeed);
			lblSpeed = new Label { Text = "500", Location = new Point(270, 25), AutoSize = true };
			grpControl.Controls.Add(lblSpeed);

			grpControl.Controls.Add(new Label { Text = "Force:", Location = new Point(10, 60), AutoSize = true });
			trackForce = new TrackBar { Minimum = 0, Maximum = 3000, Value = 1000, Location = new Point(60, 55), Size = new Size(200, 45), TickFrequency = 300 };
			trackForce.ValueChanged += TrackForce_ValueChanged;
			grpControl.Controls.Add(trackForce);
			lblForce = new Label { Text = "1000g", Location = new Point(270, 60), AutoSize = true };
			grpControl.Controls.Add(lblForce);

			// Quick Actions Group
			GroupBox grpActions = new GroupBox { Text = "Quick Actions", Location = new Point(730, 10), Size = new Size(500, 100) };
			this.Controls.Add(grpActions);

			btnOpenHand = new Button { Text = "Release Grip", Location = new Point(10, 25), Size = new Size(100, 30) };
			btnOpenHand.Click += BtnOpenHand_Click;
			grpActions.Controls.Add(btnOpenHand);

			btnCloseHand = new Button { Text = "Grip Object", Location = new Point(120, 25), Size = new Size(100, 30) };
			btnCloseHand.Click += BtnCloseHand_Click;
			grpActions.Controls.Add(btnCloseHand);

			btnStartTracking = new Button { Text = "Start Tracking", Location = new Point(10, 60), Size = new Size(100, 30), BackColor = Color.FromArgb(0, 120, 200), ForeColor = Color.White, FlatStyle = FlatStyle.Flat };
			btnStartTracking.Click += BtnStartTracking_Click;
			grpActions.Controls.Add(btnStartTracking);

			btnStopTracking = new Button { Text = "Stop Tracking", Location = new Point(120, 60), Size = new Size(100, 30), Enabled = false };
			btnStopTracking.Click += BtnStopTracking_Click;
			grpActions.Controls.Add(btnStopTracking);

			chkMirrorControl = new CheckBox { Text = "Send Tracking to Hand", Location = new Point(240, 30), AutoSize = true, Checked = false };
			grpActions.Controls.Add(chkMirrorControl);
			grpActions.Controls.Add(new Label { Text = "(Uses Python + MediaPipe)", Location = new Point(240, 55), AutoSize = true, ForeColor = Color.Gray });

			CheckBox chkLeftHand = new CheckBox { Text = "Left Hand", Location = new Point(240, 75), AutoSize = true, Checked = false };
			chkLeftHand.CheckedChanged += (s, e) => panel3DObject.IsLeftHand = chkLeftHand.Checked;
			grpActions.Controls.Add(chkLeftHand);

			// Finger Sliders Group
			GroupBox grpFingers = new GroupBox { Text = "Finger Control (Target / Actual)", Location = new Point(10, 120), Size = new Size(350, 250) };
			this.Controls.Add(grpFingers);

			for (int i = 0; i < 6; i++)
			{
				grpFingers.Controls.Add(new Label { Text = FINGER_NAMES[i] + ":", Location = new Point(10, 25 + i * 35), Size = new Size(80, 20) });
				TrackBar slider = new TrackBar { Minimum = 0, Maximum = 1000, Value = 1000, Location = new Point(90, 20 + i * 35), Size = new Size(150, 45), TickFrequency = 100, Tag = i };
				slider.ValueChanged += FingerSlider_ValueChanged;
				fingerSliders[i] = slider;
				grpFingers.Controls.Add(slider);
				fingerLabels[i] = new Label { Text = "1000", Location = new Point(245, 25 + i * 35), Size = new Size(40, 20) };
				grpFingers.Controls.Add(fingerLabels[i]);
				actualAngleLabels[i] = new Label { Text = "(---)", Location = new Point(285, 25 + i * 35), Size = new Size(55, 20), ForeColor = Color.Blue };
				grpFingers.Controls.Add(actualAngleLabels[i]);
			}

			// Tracking Panel
			GroupBox grpTracking = new GroupBox { Text = "Hand Tracking Visualization", Location = new Point(370, 120), Size = new Size(350, 350) };
			this.Controls.Add(grpTracking);
			panelTracking = new DoubleBufferedPanel { Location = new Point(10, 20), Size = new Size(330, 320), BackColor = Color.FromArgb(20, 20, 25) };
			panelTracking.Paint += PanelTracking_Paint;
			grpTracking.Controls.Add(panelTracking);

			// Tactile Panel
			GroupBox grpTactile = new GroupBox { Text = "Tactile Sensors", Location = new Point(730, 120), Size = new Size(500, 350) };
			this.Controls.Add(grpTactile);
			panelTactile = new Panel { Location = new Point(10, 20), Size = new Size(480, 320), BackColor = Color.FromArgb(252, 252, 252), BorderStyle = BorderStyle.FixedSingle };
			grpTactile.Controls.Add(panelTactile);
			SetupTactileGrids();

			// 3D Object Visualization (moved to bottom)
			GroupBox grpObjectVis = new GroupBox { Text = "3D Object Recognition", Location = new Point(10, 480), Size = new Size(1450, 390), Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom };
			this.Controls.Add(grpObjectVis);
			panel3DObject = new Object3DVisualizationPanel { Location = new Point(10, 20), Size = new Size(1430, 360), Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom };
			grpObjectVis.Controls.Add(panel3DObject);

			// Log (hidden)
			txtLog = new TextBox { Multiline = true, ReadOnly = true, ScrollBars = ScrollBars.Vertical, Location = new Point(10, 20), Size = new Size(1430, 360), Anchor = AnchorStyles.Top | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Bottom, Font = new Font("Consolas", 9), Visible = false };
			this.Controls.Add(txtLog);

			Log("=== Inspire Robotics Dexterous Hand Controller ===");
			Log("Now using TACTILE CURVATURE ANALYSIS for shape recognition!");
			Log("- Analyzes pressure gradients within each sensor");
			Log("- High center pressure + falloff = Sphere/Curved surface");
			Log("- Uniform pressure = Flat surface");
			Log("");
		}

		private void SetupTactileGrids()
		{
			int fingerWidth = 55, fingerGap = 8, tipHeight = 25, nailHeight = 65, padHeight = 55, sectionGap = 3, startY = 22, labelHeight = 16;
			int fourFingersWidth = fingerWidth * 4 + fingerGap * 3, startX = 12;
			int[] fingerX = new int[4];
			for (int i = 0; i < 4; i++) fingerX[i] = startX + i * (fingerWidth + fingerGap);

			AddFingerLabel("Little", fingerX[0], startY - labelHeight, fingerWidth);
			gridLittleTip = CreateAndAddGrid(3, 3, fingerX[0], startY, fingerWidth, tipHeight);
			gridLittleNail = CreateAndAddGrid(12, 8, fingerX[0], startY + tipHeight + sectionGap, fingerWidth, nailHeight);
			gridLittlePad = CreateAndAddGrid(10, 8, fingerX[0], startY + tipHeight + nailHeight + sectionGap * 2, fingerWidth, padHeight);

			AddFingerLabel("Ring", fingerX[1], startY - labelHeight, fingerWidth);
			gridRingTip = CreateAndAddGrid(3, 3, fingerX[1], startY, fingerWidth, tipHeight);
			gridRingNail = CreateAndAddGrid(12, 8, fingerX[1], startY + tipHeight + sectionGap, fingerWidth, nailHeight);
			gridRingPad = CreateAndAddGrid(10, 8, fingerX[1], startY + tipHeight + nailHeight + sectionGap * 2, fingerWidth, padHeight);

			AddFingerLabel("Middle", fingerX[2], startY - labelHeight, fingerWidth);
			gridMiddleTip = CreateAndAddGrid(3, 3, fingerX[2], startY, fingerWidth, tipHeight);
			gridMiddleNail = CreateAndAddGrid(12, 8, fingerX[2], startY + tipHeight + sectionGap, fingerWidth, nailHeight);
			gridMiddlePad = CreateAndAddGrid(10, 8, fingerX[2], startY + tipHeight + nailHeight + sectionGap * 2, fingerWidth, padHeight);

			AddFingerLabel("Index", fingerX[3], startY - labelHeight, fingerWidth);
			gridIndexTip = CreateAndAddGrid(3, 3, fingerX[3], startY, fingerWidth, tipHeight);
			gridIndexNail = CreateAndAddGrid(12, 8, fingerX[3], startY + tipHeight + sectionGap, fingerWidth, nailHeight);
			gridIndexPad = CreateAndAddGrid(10, 8, fingerX[3], startY + tipHeight + nailHeight + sectionGap * 2, fingerWidth, padHeight);

			int fingerBottom = startY + tipHeight + nailHeight + padHeight + sectionGap * 2;
			int palmY = fingerBottom + 18, palmHeight = 107;

			AddFingerLabel("Palm", startX + fourFingersWidth / 2 - 15, palmY - labelHeight + 2, 30);
			gridPalm = CreateAndAddGrid(8, 14, startX, palmY, fourFingersWidth, palmHeight);

			int thumbX = startX + fourFingersWidth + fingerGap * 2 + 8, thumbStartY = startY + 35;
			AddFingerLabel("Thumb", thumbX, thumbStartY - labelHeight, fingerWidth);
			gridThumbTip = CreateAndAddGrid(3, 3, thumbX, thumbStartY, fingerWidth, 22);
			gridThumbNail = CreateAndAddGrid(12, 8, thumbX, thumbStartY + 25, fingerWidth, 55);
			gridThumbMid = CreateAndAddGrid(3, 3, thumbX, thumbStartY + 83, fingerWidth, 22);
			gridThumbPad = CreateAndAddGrid(12, 8, thumbX, thumbStartY + 108, fingerWidth, 55);

			AddPressureLegend(thumbX + fingerWidth + 12, thumbStartY + 20);
		}

		private TactileSensorGrid CreateAndAddGrid(int rows, int cols, int x, int y, int width, int height)
		{
			var grid = new TactileSensorGrid(rows, cols) { Location = new Point(x, y), Size = new Size(width, height) };
			panelTactile.Controls.Add(grid);
			return grid;
		}

		private void AddFingerLabel(string text, int x, int y, int width)
		{
			panelTactile.Controls.Add(new Label { Text = text, Location = new Point(x, y), Size = new Size(width, 15), TextAlign = ContentAlignment.MiddleCenter, Font = new Font("Segoe UI", 8, FontStyle.Bold), ForeColor = Color.FromArgb(70, 70, 70) });
		}

		private void AddPressureLegend(int x, int y)
		{
			panelTactile.Controls.Add(new Label { Text = "Pressure", Location = new Point(x, y), AutoSize = true, Font = new Font("Segoe UI", 8, FontStyle.Bold) });
			var legendPanel = new Panel { Location = new Point(x, y + 18), Size = new Size(25, 120), BorderStyle = BorderStyle.FixedSingle };
			legendPanel.Paint += (s, e) =>
			{
				using (var brush = new LinearGradientBrush(new Point(0, 0), new Point(0, legendPanel.Height), Color.Red, Color.FromArgb(225, 225, 225)))
				{
					ColorBlend blend = new ColorBlend(4) { Colors = new[] { Color.Red, Color.Yellow, Color.Green, Color.FromArgb(225, 225, 225) }, Positions = new[] { 0f, 0.33f, 0.66f, 1f } };
					brush.InterpolationColors = blend;
					e.Graphics.FillRectangle(brush, 0, 0, legendPanel.Width, legendPanel.Height);
				}
			};
			panelTactile.Controls.Add(legendPanel);
			panelTactile.Controls.Add(new Label { Text = "High", Location = new Point(x + 30, y + 18), AutoSize = true, Font = new Font("Segoe UI", 7) });
			panelTactile.Controls.Add(new Label { Text = "Low", Location = new Point(x + 30, y + 125), AutoSize = true, Font = new Font("Segoe UI", 7) });
		}

		private void SetupTimers()
		{
			updateTimer = new System.Windows.Forms.Timer { Interval = 33 };
			updateTimer.Tick += UpdateTimer_Tick;
			updateTimer.Start();

			_commandProcessTimer = new System.Windows.Forms.Timer { Interval = 50 };
			_commandProcessTimer.Tick += CommandProcessTimer_Tick;
			_commandProcessTimer.Start();
		}

		private async void CommandProcessTimer_Tick(object sender, EventArgs e)
		{
			if (!_isConnected) return;
			short[] latestCommand = null;
			while (_angleCommandQueue.TryDequeue(out var cmd)) latestCommand = cmd;
			if (latestCommand != null) try { await SetAnglesAsync(latestCommand); } catch { }
		}

		private void Log(string message)
		{
			if (InvokeRequired) { try { BeginInvoke(new Action<string>(Log), message); } catch { } return; }
			txtLog?.AppendText("[" + DateTime.Now.ToString("HH:mm:ss") + "] " + message + Environment.NewLine);
		}

		private async void BtnConnect_Click(object sender, EventArgs e)
		{
			try
			{
				_ipAddress = txtIpAddress.Text;
				_port = (int)numPort.Value;
				Log("Connecting to " + _ipAddress + ":" + _port + "...");

				_tcpClient = new TcpClient { ReceiveTimeout = 3000, SendTimeout = 3000 };
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

		private void BtnDisconnect_Click(object sender, EventArgs e) => Disconnect();

		private void Disconnect()
		{
			StopTactileReading();
			_isConnected = false;
			_stream?.Close(); _stream = null;
			_tcpClient?.Close(); _tcpClient = null;

			lblStatus.Text = "● Disconnected";
			lblStatus.ForeColor = Color.Red;
			btnConnect.Enabled = true;
			btnDisconnect.Enabled = false;

			ClearAllTactileGrids();
			for (int i = 0; i < 6; i++) actualAngleLabels[i].Text = "(---)";
			Log("Disconnected.");
		}

		private void ClearAllTactileGrids()
		{
			gridLittleTip?.ClearData(); gridLittleNail?.ClearData(); gridLittlePad?.ClearData();
			gridRingTip?.ClearData(); gridRingNail?.ClearData(); gridRingPad?.ClearData();
			gridMiddleTip?.ClearData(); gridMiddleNail?.ClearData(); gridMiddlePad?.ClearData();
			gridIndexTip?.ClearData(); gridIndexNail?.ClearData(); gridIndexPad?.ClearData();
			gridThumbTip?.ClearData(); gridThumbNail?.ClearData(); gridThumbMid?.ClearData(); gridThumbPad?.ClearData();
			gridPalm?.ClearData();
			panel3DObject?.Clear();
		}

		private async Task<int[]> ReadHoldingRegistersAsync(int startAddress, int quantity)
		{
			if (!_isConnected || _stream == null) throw new InvalidOperationException("Not connected");

			await _modbusSemaphore.WaitAsync();
			try
			{
				ushort currentTransactionId;
				lock (_streamLock) { _transactionId++; currentTransactionId = _transactionId; }

				byte[] request = new byte[12];
				request[0] = (byte)(currentTransactionId >> 8); request[1] = (byte)(currentTransactionId & 0xFF);
				request[2] = 0x00; request[3] = 0x00; request[4] = 0x00; request[5] = 0x06;
				request[6] = 0xFF; request[7] = 0x03;
				request[8] = (byte)(startAddress >> 8); request[9] = (byte)(startAddress & 0xFF);
				request[10] = (byte)(quantity >> 8); request[11] = (byte)(quantity & 0xFF);

				await _stream.WriteAsync(request, 0, request.Length);

				byte[] header = new byte[9];
				int totalRead = 0;
				while (totalRead < 9) { int bytesRead = await _stream.ReadAsync(header, totalRead, 9 - totalRead); if (bytesRead == 0) throw new Exception("Connection closed"); totalRead += bytesRead; }

				int byteCount = header[8];
				byte[] data = new byte[byteCount];
				totalRead = 0;
				while (totalRead < byteCount) { int bytesRead = await _stream.ReadAsync(data, totalRead, byteCount - totalRead); if (bytesRead == 0) throw new Exception("Connection closed"); totalRead += bytesRead; }

				int[] registers = new int[quantity];
				for (int i = 0; i < quantity && i * 2 + 1 < data.Length; i++) registers[i] = (data[i * 2] << 8) | data[i * 2 + 1];
				return registers;
			}
			finally { _modbusSemaphore.Release(); }
		}

		private async Task WriteMultipleRegistersAsync(int startAddress, int[] values)
		{
			if (!_isConnected || _stream == null) return;

			await _modbusSemaphore.WaitAsync();
			try
			{
				ushort currentTransactionId;
				lock (_streamLock) { _transactionId++; currentTransactionId = _transactionId; }

				int quantity = values.Length;
				byte byteCount = (byte)(quantity * 2);
				byte[] request = new byte[13 + byteCount];
				request[0] = (byte)(currentTransactionId >> 8); request[1] = (byte)(currentTransactionId & 0xFF);
				request[2] = 0x00; request[3] = 0x00;
				request[4] = (byte)((7 + byteCount) >> 8); request[5] = (byte)((7 + byteCount) & 0xFF);
				request[6] = 0xFF; request[7] = 0x10;
				request[8] = (byte)(startAddress >> 8); request[9] = (byte)(startAddress & 0xFF);
				request[10] = (byte)(quantity >> 8); request[11] = (byte)(quantity & 0xFF);
				request[12] = byteCount;

				for (int i = 0; i < quantity; i++) { request[13 + i * 2] = (byte)(values[i] >> 8); request[14 + i * 2] = (byte)(values[i] & 0xFF); }

				await _stream.WriteAsync(request, 0, request.Length);
				byte[] response = new byte[12];
				await _stream.ReadAsync(response, 0, 12);
			}
			finally { _modbusSemaphore.Release(); }
		}

		private async Task<short[]> ReadActualAnglesAsync()
		{
			try
			{
				int[] data = await ReadHoldingRegistersAsync(ANGLE_ACT_ADDRESS, 6);
				short[] angles = new short[6];
				for (int i = 0; i < 6 && i < data.Length; i++) angles[i] = (short)data[i];
				return angles;
			}
			catch { return _actualAnglesFromRobot; }
		}

		private async Task SetAnglesAsync(short[] angles)
		{
			if (!_isConnected) return;
			int[] values = new int[6];
			for (int i = 0; i < 6; i++) values[i] = ClampShort(angles[i], 0, 1000);
			await WriteMultipleRegistersAsync(1486, values);
		}

		private async Task SetGlobalSpeedAsync(short speed)
		{
			if (!_isConnected) return;
			int[] values = new int[6];
			for (int i = 0; i < 6; i++) values[i] = ClampShort(speed, 0, 1000);
			await WriteMultipleRegistersAsync(1522, values);
			Log("Speed set to " + speed);
		}

		private async Task SetGlobalForceAsync(short force)
		{
			if (!_isConnected) return;
			int[] values = new int[6];
			for (int i = 0; i < 6; i++) values[i] = ClampShort(force, 0, 3000);
			await WriteMultipleRegistersAsync(1498, values);
			Log("Force set to " + force + "g");
		}

		private void StartTactileReading()
		{
			if (_isReadingTactile) return;
			_isReadingTactile = true;
			_tactileCts = new CancellationTokenSource();
			Task.Run(() => TactileReadLoop(_tactileCts.Token));
			Log("Tactile sensor and angle reading started");
		}

		private void StopTactileReading()
		{
			_isReadingTactile = false;
			_tactileCts?.Cancel();
			_tactileCts = null;
		}

		private async Task TactileReadLoop(CancellationToken token)
		{
			while (_isReadingTactile && !token.IsCancellationRequested && _isConnected)
			{
				try
				{
					var actualAngles = await ReadActualAnglesAsync();
					_actualAnglesFromRobot = actualAngles;

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

					_lastLittleTip = littleTip; _lastLittlePad = littlePad;
					_lastRingTip = ringTip; _lastRingPad = ringPad;
					_lastMiddleTip = middleTip; _lastMiddlePad = middlePad;
					_lastIndexTip = indexTip; _lastIndexPad = indexPad;
					_lastThumbTip = thumbTip; _lastThumbPad = thumbPad;
					_lastPalm = palmRemapped;

					if (!token.IsCancellationRequested)
					{
						try
						{
							this.BeginInvoke(new Action(() =>
							{
								gridLittleTip?.UpdateData(littleTip); gridLittleNail?.UpdateData(littleNail); gridLittlePad?.UpdateData(littlePad);
								gridRingTip?.UpdateData(ringTip); gridRingNail?.UpdateData(ringNail); gridRingPad?.UpdateData(ringPad);
								gridMiddleTip?.UpdateData(middleTip); gridMiddleNail?.UpdateData(middleNail); gridMiddlePad?.UpdateData(middlePad);
								gridIndexTip?.UpdateData(indexTip); gridIndexNail?.UpdateData(indexNail); gridIndexPad?.UpdateData(indexPad);
								gridThumbTip?.UpdateData(thumbTip); gridThumbNail?.UpdateData(thumbNail); gridThumbMid?.UpdateData(thumbMid); gridThumbPad?.UpdateData(thumbPad);
								gridPalm?.UpdateData(palmRemapped);

								for (int i = 0; i < 6; i++)
								{
									actualAngleLabels[i].Text = $"({actualAngles[i]})";
									int diff = Math.Abs(fingerSliders[i].Value - actualAngles[i]);
									actualAngleLabels[i].ForeColor = diff < 20 ? Color.Green : (diff < 100 ? Color.Orange : Color.Blue);
								}

								panel3DObject?.UpdateData(actualAngles, littleTip, littlePad, ringTip, ringPad, middleTip, middlePad, indexTip, indexPad, thumbTip, thumbPad, palmRemapped);
							}));
						}
						catch { }
					}

					await Task.Delay(100, token);
				}
				catch (OperationCanceledException) { break; }
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
				for (int i = 0; i < registersToRead && i < data.Length; i++) result[registersRead + i] = data[i];
				registersRead += registersToRead;
			}
			return result;
		}

		private int[] RemapPalmData(int[] rawData)
		{
			int rows = 8, cols = 14;
			int[] remapped = new int[rows * cols];
			for (int col = 0; col < cols; col++)
				for (int row = 0; row < rows; row++)
				{
					int sourceIndex = col * rows + (rows - 1 - row);
					int destIndex = (rows - 1 - row) * cols + col;
					if (sourceIndex < rawData.Length && destIndex < remapped.Length) remapped[destIndex] = rawData[sourceIndex];
				}
			return remapped;
		}

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
			Log("Releasing grip...");
			for (int i = 0; i < 6; i++) { _targetAngles[i] = 1000; fingerSliders[i].Value = 1000; }
			await SetAnglesAsync(_targetAngles);
		}

		private async void BtnCloseHand_Click(object sender, EventArgs e)
		{
			Log("Gripping object...");
			
			// First, move thumb rotation to 0 position
			_targetAngles[5] = 0;
			fingerSliders[5].Value = 0;
			await SetAnglesAsync(_targetAngles);
			
			// Wait a moment for thumb to rotate
			await Task.Delay(500);
			
			// Then bend all DOFs
			for (int i = 0; i < 5; i++) _targetAngles[i] = 0;
			for (int i = 0; i < 6; i++) fingerSliders[i].Value = _targetAngles[i];
			await SetAnglesAsync(_targetAngles);
		}

		private async void FingerSlider_ValueChanged(object sender, EventArgs e)
		{
			if (_isUpdatingSliders) return;
			TrackBar slider = sender as TrackBar;
			int idx = (int)slider.Tag;
			_targetAngles[idx] = (short)slider.Value;
			fingerLabels[idx].Text = slider.Value.ToString();
			if (_isConnected && !chkMirrorControl.Checked) await SetAnglesAsync(_targetAngles);
		}

		private void BtnStartTracking_Click(object sender, EventArgs e) => StartTracking();
		private void BtnStopTracking_Click(object sender, EventArgs e) => StopTracking();

		private void StartTracking()
		{
			try
			{
				_udpClient = new UdpClient(5065) { Client = { ReceiveTimeout = 100 } };
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
				if (!System.IO.File.Exists(scriptPath)) { Log("ERROR: hand_tracker.py not found at " + scriptPath); return; }
				Log("Starting Python hand tracker...");
				_pythonProcess = Process.Start(new ProcessStartInfo { FileName = "py", Arguments = "-3.11 \"" + scriptPath + "\"", UseShellExecute = true, WorkingDirectory = AppDomain.CurrentDomain.BaseDirectory });
				Log("Python hand tracker started");
			}
			catch (Exception ex) { Log("Failed to start Python: " + ex.Message); }
		}

		private void StopTracking()
		{
			_isTrackingRunning = false;
			_trackingCts?.Cancel(); _trackingCts = null;
			try { _udpClient?.Close(); } catch { }
			_udpClient = null;
			if (_pythonProcess != null && !_pythonProcess.HasExited) try { _pythonProcess.Kill(); } catch { }
			_pythonProcess = null;
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
					if (_udpClient?.Available > 0)
					{
						byte[] data = _udpClient.Receive(ref remoteEP);
						ProcessTrackingData(Encoding.UTF8.GetString(data));
					}
					else Thread.Sleep(5);
				}
				catch (SocketException) { Thread.Sleep(10); }
				catch (ObjectDisposedException) { break; }
				catch { Thread.Sleep(10); }
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
						for (int i = 0; i < 6 && i < anglesArray.Count; i++)
							_trackedAngles[i] = ClampShort(anglesArray[i].Value<int>(), 0, 1000);

					JArray landmarksArray = data["landmarks"] as JArray;
					if (landmarksArray?.Count >= 21)
					{
						_handLandmarks = new float[landmarksArray.Count][];
						for (int i = 0; i < landmarksArray.Count; i++)
						{
							JObject lm = landmarksArray[i] as JObject;
							if (lm != null) _handLandmarks[i] = new[] { lm["x"]?.Value<float>() ?? 0, lm["y"]?.Value<float>() ?? 0, lm["z"]?.Value<float>() ?? 0 };
						}
					}

					if (chkMirrorControl.Checked)
					{
						_angleCommandQueue.Enqueue((short[])_trackedAngles.Clone());
						try
						{
							this.BeginInvoke(new Action(() =>
							{
								_isUpdatingSliders = true;
								for (int i = 0; i < 6; i++) { fingerSliders[i].Value = _trackedAngles[i]; fingerLabels[i].Text = _trackedAngles[i].ToString(); _targetAngles[i] = _trackedAngles[i]; }
								_isUpdatingSliders = false;
							}));
						}
						catch { }
					}
				}
			}
			catch { }
		}

		private void PanelTracking_Paint(object sender, PaintEventArgs e)
		{
			Graphics g = e.Graphics;
			g.SmoothingMode = SmoothingMode.AntiAlias;
			int w = panelTracking.Width, h = panelTracking.Height;

			using (Font font = new Font("Segoe UI", 10, FontStyle.Bold))
			using (Font smallFont = new Font("Segoe UI", 9))
			{
				if (!_isTrackingRunning) { g.DrawString("Click 'Start Tracking' to begin", font, Brushes.White, 10, 10); return; }
				if (!_handDetected || (DateTime.Now - _lastDataReceived).TotalSeconds >= 2) { g.DrawString("Waiting for hand data...", font, Brushes.Yellow, 10, 10); return; }

				if (_handLandmarks?.Length >= 21) DrawHandSkeleton(g, w, h);

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
				if (chkMirrorControl.Checked) g.DrawString("✓ Sending to robot", smallFont, Brushes.Cyan, 10, y + 5);
			}
		}

		private void DrawHandSkeleton(Graphics g, int w, int h)
		{
			if (_handLandmarks == null) return;

			int[][] connections = { new[] { 0, 1 }, new[] { 1, 2 }, new[] { 2, 3 }, new[] { 3, 4 }, new[] { 0, 5 }, new[] { 5, 6 }, new[] { 6, 7 }, new[] { 7, 8 }, new[] { 0, 9 }, new[] { 9, 10 }, new[] { 10, 11 }, new[] { 11, 12 }, new[] { 0, 13 }, new[] { 13, 14 }, new[] { 14, 15 }, new[] { 15, 16 }, new[] { 0, 17 }, new[] { 17, 18 }, new[] { 18, 19 }, new[] { 19, 20 }, new[] { 5, 9 }, new[] { 9, 13 }, new[] { 13, 17 } };

			float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
			foreach (var lm in _handLandmarks) if (lm != null) { minX = Math.Min(minX, lm[0]); maxX = Math.Max(maxX, lm[0]); minY = Math.Min(minY, lm[1]); maxY = Math.Max(maxY, lm[1]); }

			float rangeX = Math.Max(0.001f, maxX - minX), rangeY = Math.Max(0.001f, maxY - minY);
			float scale = Math.Min((w - 140) / rangeX, (h - 40) / rangeY);
			float offsetX = (w - rangeX * scale) / 2 + 60, offsetY = (h - rangeY * scale) / 2;

			using (Pen pen = new Pen(Color.White, 2))
				foreach (var conn in connections)
					if (conn[0] < _handLandmarks.Length && conn[1] < _handLandmarks.Length && _handLandmarks[conn[0]] != null && _handLandmarks[conn[1]] != null)
						g.DrawLine(pen, offsetX + (_handLandmarks[conn[0]][0] - minX) * scale, offsetY + (_handLandmarks[conn[0]][1] - minY) * scale, offsetX + (_handLandmarks[conn[1]][0] - minX) * scale, offsetY + (_handLandmarks[conn[1]][1] - minY) * scale);

			using (SolidBrush brush = new SolidBrush(Color.LightGreen))
				foreach (var lm in _handLandmarks)
					if (lm != null) g.FillEllipse(brush, offsetX + (lm[0] - minX) * scale - 4, offsetY + (lm[1] - minY) * scale - 4, 8, 8);
		}

		private void UpdateTimer_Tick(object sender, EventArgs e)
		{
			panelTracking.Invalidate();
			if (!_isConnected && _isTrackingRunning && _handDetected)
				panel3DObject?.UpdateData(_trackedAngles, _lastLittleTip ?? new int[9], _lastLittlePad ?? new int[80], _lastRingTip ?? new int[9], _lastRingPad ?? new int[80], _lastMiddleTip ?? new int[9], _lastMiddlePad ?? new int[80], _lastIndexTip ?? new int[9], _lastIndexPad ?? new int[80], _lastThumbTip ?? new int[9], _lastThumbPad ?? new int[96], _lastPalm ?? new int[112]);
		}

		protected override void OnFormClosing(FormClosingEventArgs e)
		{
			_commandProcessTimer?.Stop(); _commandProcessTimer?.Dispose();
			StopTactileReading();
			StopTracking();
			Disconnect();
			updateTimer?.Stop(); updateTimer?.Dispose();
			base.OnFormClosing(e);
		}
	}

	#endregion
}
