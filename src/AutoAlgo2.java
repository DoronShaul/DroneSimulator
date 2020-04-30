import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.util.ArrayList;

import javafx.util.Pair;

public class AutoAlgo2 {

	int map_size = 3000;

	enum PixelState {
		blocked, explored, unexplored, visited
	};

	PixelState map[][];
	Drone drone;
	Point droneStartingPoint;

	ArrayList<Point> points;
	ArrayList<Pair<Point, String>> riskPoints;

	int isRotating;
	ArrayList<Double> degrees_left;
	ArrayList<Func> degrees_left_func;

	boolean isSpeedUp = false;

	Graph mGraph = new Graph();

	CPU ai_cpu;

	public AutoAlgo2(Map realMap) {
		degrees_left = new ArrayList<>();
		degrees_left_func = new ArrayList<>();
		points = new ArrayList<Point>();
		riskPoints = new ArrayList<Pair<Point, String>>();

		drone = new Drone(realMap);
		drone.addLidar(0);
		drone.addLidar(90);
		drone.addLidar(-90);

		initMap();

		isRotating = 0;
		ai_cpu = new CPU(200, "Auto_AI");
		ai_cpu.addFunction(this::update);
	}

	public void initMap() {
		map = new PixelState[map_size][map_size];
		for (int i = 0; i < map_size; i++) {
			for (int j = 0; j < map_size; j++) {
				map[i][j] = PixelState.unexplored;
			}
		}

		droneStartingPoint = new Point(map_size / 2, map_size / 2);
	}

	public void play() {
		drone.play();
		ai_cpu.play();
	}

	public void update(int deltaTime) {
		updateVisited();
		updateMapByLidars();

		ai(deltaTime);

		if (isRotating != 0) {
			updateRotating(deltaTime);
		}
		if (isSpeedUp) {
			drone.speedUp(deltaTime);
		} else {
			drone.slowDown(deltaTime);
		}

	}

	public void speedUp() {
		isSpeedUp = true;
	}

	public void speedDown() {
		isSpeedUp = false;
	}

	public void updateMapByLidars() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x, dronePoint.y + droneStartingPoint.y);

		for (int i = 0; i < drone.lidars.size(); i++) {
			Lidar lidar = drone.lidars.get(i);
			double rotation = drone.getGyroRotation() + lidar.degrees;
			// rotation = Drone.formatRotation(rotation);
			for (int distanceInCM = 0; distanceInCM < lidar.current_distance; distanceInCM++) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, distanceInCM);
				setPixel(p.x, p.y, PixelState.explored);
			}

			if (lidar.current_distance > 0
					&& lidar.current_distance < WorldParams.lidarLimit - WorldParams.lidarNoise) {
				Point p = Tools.getPointByDistance(fromPoint, rotation, lidar.current_distance);
				setPixel(p.x, p.y, PixelState.blocked);
				// fineEdges((int)p.x,(int)p.y);
			}
		}
	}

	public void updateVisited() {
		Point dronePoint = drone.getOpticalSensorLocation();
		Point fromPoint = new Point(dronePoint.x + droneStartingPoint.x, dronePoint.y + droneStartingPoint.y);

		setPixel(fromPoint.x, fromPoint.y, PixelState.visited);

	}

	public void setPixel(double x, double y, PixelState state) {
		int xi = (int) x;
		int yi = (int) y;

		if (state == PixelState.visited) {
			map[xi][yi] = state;
			return;
		}

		if (map[xi][yi] == PixelState.unexplored) {
			map[xi][yi] = state;
		}
	}

	public void paintBlindMap(Graphics g) {
		Color c = g.getColor();

		int i = (int) droneStartingPoint.y - (int) drone.startPoint.x;
		int startY = i;
		for (; i < map_size; i++) {
			int j = (int) droneStartingPoint.x - (int) drone.startPoint.y;
			int startX = j;
			for (; j < map_size; j++) {
				if (map[i][j] != PixelState.unexplored) {
					if (map[i][j] == PixelState.blocked) {
						g.setColor(Color.RED);
					} else if (map[i][j] == PixelState.explored) {
						g.setColor(Color.YELLOW);
					} else if (map[i][j] == PixelState.visited) {
						g.setColor(Color.BLUE);
					}
					g.drawLine(i - startY, j - startX, i - startY, j - startX);
				}
			}
		}
		g.setColor(c);
	}

	public void paintPoints(Graphics g) {
		for (int i = 0; i < points.size(); i++) {
			Point p = points.get(i);
			Color color = Color.BLACK;
			g.setColor(color);
			g.drawOval((int) p.x + (int) drone.startPoint.x - 10, (int) p.y + (int) drone.startPoint.y - 10, 20, 20);

		}

	}

	public void PaintProblems(Graphics g) {
		for (int i = 0; i < riskPoints.size(); i++) {
			Pair<Point, String> pair = riskPoints.get(i);
			if (pair.getValue() == "front") {
				Color color = Color.CYAN;
				g.setColor(color);
				g.fillOval((int) pair.getKey().x + (int) drone.startPoint.x - 10,
						(int) pair.getKey().y + (int) drone.startPoint.y - 10, 5, 5);
			} else if (pair.getValue() == "right") {
				Color color = Color.BLACK;
				g.setColor(color);
				g.fillOval((int) pair.getKey().x + (int) drone.startPoint.x - 10,
						(int) pair.getKey().y + (int) drone.startPoint.y - 10, 5, 5);
			} else if (pair.getValue() == "left") {
				Color color = Color.RED;
				g.setColor(color);
				g.fillOval((int) pair.getKey().x + (int) drone.startPoint.x - 10,
						(int) pair.getKey().y + (int) drone.startPoint.y - 10, 5, 5);
			} else if (pair.getValue() == "right and front") {
				Color color = Color.GREEN;
				g.setColor(color);
				g.fillOval((int) pair.getKey().x + (int) drone.startPoint.x - 10,
						(int) pair.getKey().y + (int) drone.startPoint.y - 10, 5, 5);
			} else if (pair.getValue() == "left and front") {
				Color color = Color.MAGENTA;
				g.setColor(color);
				g.fillOval((int) pair.getKey().x + (int) drone.startPoint.x - 10,
						(int) pair.getKey().y + (int) drone.startPoint.y - 10, 5, 5);
			} else if (pair.getValue() == "left and right") {
				Color color = Color.ORANGE;
				g.setColor(color);
				g.fillOval((int) pair.getKey().x + (int) drone.startPoint.x - 10,
						(int) pair.getKey().y + (int) drone.startPoint.y - 10, 5, 5);
			} else if (pair.getValue() == "all") {
				Color color = Color.PINK;
				g.setColor(color);
				g.fillOval((int) pair.getKey().x + (int) drone.startPoint.x - 10,
						(int) pair.getKey().y + (int) drone.startPoint.y - 10, 5, 5);
			}

		}

	}

	public void paint(Graphics g) {
		if (SimulationWindow.toogleRealMap) {
			drone.realMap.paint(g);
		}

		paintBlindMap(g);
		paintPoints(g);
		PaintProblems(g);

		drone.paint(g);

	}

	boolean is_init = true;
	double lastFrontLidarDis = 0;
	boolean isRotateRight = false;
	double changedRight = 0;
	double changedLeft = 0;
	boolean tryToEscape = false;
	int leftOrRight = 1;
	boolean isFrontRisky = false;
	boolean isRightRisky = false;
	boolean isLeftRisky = false;
	boolean isLeftBigger = false;
	double sideLidarsDiff = 0;
	boolean once = false;
	boolean lastRisky = false;
	Pair<Point, String> lastRiskyPoint;
	double lastRightLidarDist = 300;
	double lastLeftLidarDist = 300;

	double max_rotation_to_direction = 20;
	boolean is_finish = true;
	boolean isLeftRightRotationEnable = true;// **

	boolean is_risky = false;
	int max_risky_distance = 150;
	boolean try_to_escape = false;
	double risky_dis = 0;
	int max_angle_risky = 10;

	boolean is_lidars_max = false;

	double save_point_after_seconds = 3;

	double max_distance_between_points = 150;

	boolean start_return_home = false;

	Point init_point;

	public void ai(int deltaTime) {
		if (!SimulationWindow.toogleAI) {
			return;
		}

		if (is_init) {
			speedUp();
			Point dronePoint = drone.getOpticalSensorLocation();
			init_point = new Point(dronePoint);
			points.add(dronePoint);
			mGraph.addVertex(dronePoint);
			is_init = false;
		}

		Point dronePoint = drone.getOpticalSensorLocation();

		if (SimulationWindow.return_home) {

			if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) < max_distance_between_points) {
				if (points.size() <= 1 && Tools.getDistanceBetweenPoints(getLastPoint(),
						dronePoint) < max_distance_between_points / 5) {
					speedDown();
				} else {
					removeLastPoint();
				}
			}
		} else {

			if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points) {
				points.add(dronePoint);
				mGraph.addVertex(dronePoint);
			}
		}

		if (!is_risky) {
			speedUp();
			Lidar lidar = drone.lidars.get(0);
			if (lidar.current_distance <= max_risky_distance) {
				is_risky = true;
				risky_dis = lidar.current_distance;
				isFrontRisky = true;
			}

			Lidar lidar1 = drone.lidars.get(1);
			if (lidar1.current_distance <= 65) {
				is_risky = true;
				isRightRisky = true;
			}

			Lidar lidar2 = drone.lidars.get(2);
			if (lidar2.current_distance <= 65) {
				is_risky = true;
				isLeftRisky = true;
			}
			

			//this is when the drone took the right or left and the the opposite direction opens up.
			if (degrees_left.size() == 0 && once) {
				Pair<Point, String> p = riskPoints.get(riskPoints.size() - 1);
				if (p.getValue().equals("left") && lidar2.current_distance > 200) {
					spinBy(-90, true);
				} else if (p.getValue().equals("right") && lidar1.current_distance > 200) {
					spinBy(90, true);
				}
				once = false;
			} else if (lidar1.current_distance > 295 && lastRightLidarDist < 270) {
				speedDown();
				spinBy(90, true);
				System.out.println("90");
				
			} else if (lidar2.current_distance > 295 && lastLeftLidarDist < 270) {
				speedDown();
				spinBy(-90, true);
				System.out.println("-90");
			}
			
			lastRightLidarDist = lidar1.current_distance;
			lastLeftLidarDist = lidar2.current_distance;
			System.out.println("lidar1 dist: "+lidar1.current_distance);
			System.out.println("lidar1 last dist: "+lidar1.current_distance);


		} else {
			if (!try_to_escape) {
				try_to_escape = true;
				Lidar lidar = drone.lidars.get(0);
				double frontLidar = lidar.current_distance;
				Lidar lidar1 = drone.lidars.get(1);
				double a = lidar1.current_distance;

				Lidar lidar2 = drone.lidars.get(2);
				double b = lidar2.current_distance;

				int spin_by = max_angle_risky;
				Graphics g;
				// only front risky.
				if (isFrontRisky && degrees_left.size() == 0) {
					if (riskPoints.size() > 0) {
						lastRiskyPoint = riskPoints.get(riskPoints.size() - 1);
					}
					if (riskPoints.size() != 0 && (lastRiskyPoint.getValue().equals("right")
							|| lastRiskyPoint.getValue().equals("left"))) {
						if (lastRiskyPoint.getValue().equals("right")) {
							speedDown();
							spinBy(30, true);
						} else if (lastRiskyPoint.getValue().equals("left")) {
							speedDown();
							spinBy(-30, true);
						}

					} else if (riskPoints.size() == 0 || lastRiskyPoint.getValue().equals("front")) {
						sideLidarsDiff = lidar1.current_distance - lidar2.current_distance;
						if (sideLidarsDiff < 0) {
							isLeftBigger = true;
						} else {
							isLeftBigger = false;
						}

						if (isLeftBigger) {
							if (sideLidarsDiff > -50) {
								spin_by = -10;
								if (frontLidar < 70) {
									spin_by = -60;
								}
							} else if (sideLidarsDiff > -100) {
								spin_by = -30;
							} else {
								spin_by = -60;
							}
						} else {
							if (sideLidarsDiff < 50) {
								spin_by = 10;
								if (frontLidar < 70) {
									spin_by = 60;
								}
							} else if (sideLidarsDiff < 100) {
								spin_by = 30;
							} else {
								spin_by = 60;
							}
						}
						speedDown();
						spinBy(spin_by, true);
					}
					Pair<Point, String> p = new Pair<Point, String>(dronePoint, "front");
					riskPoints.add(p);

				}
				// only right risky.
				else if (isRightRisky && !isLeftRisky && degrees_left.size() == 0) {
					Pair<Point, String> p = new Pair<Point, String>(dronePoint, "right");
					riskPoints.add(p);
					speedDown();
					spinBy(-5, true);
					spinBy(3.5);
					once = true;
				}
				// only left.
				else if (!isRightRisky && isLeftRisky && degrees_left.size() == 0) {
					Pair<Point, String> p = new Pair<Point, String>(dronePoint, "left");
					riskPoints.add(p);
					speedDown();
					spinBy(5, true);
					spinBy(-3.5);
					once = true;
				}

				try_to_escape = false;
				is_risky = false;
				isFrontRisky = false;
				isRightRisky = false;
				isLeftRisky = false;
			}
		}

		// }
	}

	int counter = 0;

//	public void doLeftRight() {
//		if (is_finish) {
//			leftOrRight *= -1;
//			counter++;
//			is_finish = false;
//
//			spinBy(max_rotation_to_direction * leftOrRight, false, new Func() {
//				@Override
//				public void method() {
//					is_finish = true;
//				}
//			});
//		}
//	}

	double lastGyroRotation = 0;

	public void updateRotating(int deltaTime) {

		if (degrees_left.size() == 0) {
			return;
		}

		double degrees_left_to_rotate = degrees_left.get(0);
		boolean isLeft = true;
		if (degrees_left_to_rotate > 0) {
			isLeft = false;
		}

		double curr = drone.getGyroRotation();
		double just_rotated = 0;

		if (isLeft) {

			just_rotated = curr - lastGyroRotation;
			if (just_rotated > 0) {
				just_rotated = -(360 - just_rotated);
			}
		} else {
			just_rotated = curr - lastGyroRotation;
			if (just_rotated < 0) {
				just_rotated = 360 + just_rotated;
			}
		}

		lastGyroRotation = curr;
		degrees_left_to_rotate -= just_rotated;
		degrees_left.remove(0);
		degrees_left.add(0, degrees_left_to_rotate);

		if ((isLeft && degrees_left_to_rotate >= 0) || (!isLeft && degrees_left_to_rotate <= 0)) {
			degrees_left.remove(0);

			Func func = degrees_left_func.get(0);
			if (func != null) {
				func.method();
			}
			degrees_left_func.remove(0);

			if (degrees_left.size() == 0) {
				isRotating = 0;
			}
			return;
		}

		int direction = (int) (degrees_left_to_rotate / Math.abs(degrees_left_to_rotate));
		drone.rotateLeft(deltaTime * direction);

	}

	public void spinBy(double degrees, boolean isFirst, Func func) {
		lastGyroRotation = drone.getGyroRotation();
		if (isFirst) {
			degrees_left.add(0, degrees);
			degrees_left_func.add(0, func);

		} else {
			degrees_left.add(degrees);
			degrees_left_func.add(func);
		}

		isRotating = 1;
	}

	public void spinBy(double degrees, boolean isFirst) {
		lastGyroRotation = drone.getGyroRotation();
		if (isFirst) {
			degrees_left.add(0, degrees);
			degrees_left_func.add(0, null);

		} else {
			degrees_left.add(degrees);
			degrees_left_func.add(null);
		}

		isRotating = 1;
	}

	public void spinBy(double degrees) {
		lastGyroRotation = drone.getGyroRotation();

		degrees_left.add(degrees);
		degrees_left_func.add(null);
		isRotating = 1;
	}

	public Point getLastPoint() {
		if (points.size() == 0) {
			return init_point;
		}

		Point p1 = points.get(points.size() - 1);
		return p1;
	}

	public Point removeLastPoint() {
		if (points.isEmpty()) {
			return init_point;
		}

		return points.remove(points.size() - 1);
	}

	public Point getAvgLastPoint() {
		if (points.size() < 2) {
			return init_point;
		}

		Point p1 = points.get(points.size() - 1);
		Point p2 = points.get(points.size() - 2);
		return new Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
	}

}
