import java.awt.Color;
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

	ArrayList<Pair<Point, Integer>> points;
	ArrayList<Pair<Point, String>> riskPoints;

	int isRotating;
	ArrayList<Double> degrees_left;
	ArrayList<Func> degrees_left_func;

	boolean isSpeedUp = false;
	boolean isSpeedUpHome = false;
	boolean slowDownSide = false;
	boolean slowDrasticFront=false;
	//boolean stop = false;
	Graph mGraph = new Graph();
	MyGraph myGraph = new MyGraph();

	CPU ai_cpu;

	public AutoAlgo2(Map realMap) {
		degrees_left = new ArrayList<>();
		degrees_left_func = new ArrayList<>();
		points = new ArrayList<Pair<Point,Integer>>();
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
		} else if (slowDownSide) {
			drone.sidesSlowDown(deltaTime);
		} 
		else if(slowDrasticFront) {
			drone.slowDrasticFront(deltaTime);
		}
		else if(isSpeedUpHome) {
			drone.speedUpReturnHome(deltaTime);
		}
//		else if(stop) {
//			drone.st
//		}
		else {
			drone.slowDown(deltaTime);
		}

	}

	public void speedUp() {
		isSpeedUp = true;
		slowDownSide = false;
		slowDrasticFront=false;
		isSpeedUpHome = false;

	}
	public void speedUpHome() {
		isSpeedUpHome = true;
		isSpeedUp = false;
		slowDownSide = false;
		slowDrasticFront=false;

	}

	public void speedDown() {
		isSpeedUp = false;
		slowDownSide = false;
		slowDrasticFront=false;
		isSpeedUpHome = false;

	}

	public void slowDownSides() {
		isSpeedUp = false;
		slowDrasticFront=false;
		slowDownSide=true;
		isSpeedUpHome = false;
	}

	public void slowDrasticFront() {
		slowDrasticFront=true;
		isSpeedUp = false;
		slowDownSide = false;
		isSpeedUpHome = false;
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
			Point p = points.get(i).getKey();
			Color color = Color.BLACK;
			g.setColor(color);
			g.drawOval((int) p.x + (int) drone.startPoint.x - 10, (int) p.y + (int) drone.startPoint.y - 10, 20, 20);

		}

	}

	/**
	 * this method paints the risky areas. cyan dot when the risk is from the front, red when it's from the left, and black if it's right. 
	 * @param g
	 */
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
			}

		}

	}

	public void paint(Graphics g) {
		if (SimulationWindow.toogleRealMap) {
			drone.realMap.paint(g);
		}

		paintBlindMap(g);
		paintPoints(g);
		//PaintProblems(g);

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
	boolean shouldSpeedUp = false;
	double sideLidarsDiff = 0;
	boolean oppositeDirectionTurn = false;
	boolean lastRisky = false;
	Pair<Point, String> lastRiskyPoint;
	double lastRightLidarDist = 300;
	double lastLeftLidarDist = 300;

	double max_rotation_to_direction = 20;
	boolean is_finish = true;
	boolean isLeftRightRotationEnable = true;

	boolean is_risky = false;
	int max_risky_distance = 150;
	boolean try_to_escape = false;
	double risky_dis = 0;

	boolean is_lidars_max = false;

	double save_point_after_seconds = 3;

	double max_distance_between_points =50 ;

	boolean start_return_home = false;

	Point init_point;
	Pair<Point, Integer> initPair;
	int countTime=0;
	public static int realTime=0;
	public static int pointWeight=0;
	boolean isRightTurn=false;
	boolean isLeftTurn=false;
	boolean addPoint=false;
	boolean returnHome=false;
	int spin_by = 0;
	boolean turnAround = true;
	boolean stopAI = false;
	double angle;
	int lastEdgeWeight = 0;
	boolean returnedHome = true;



	public void ai(int deltaTime) {

		if (!SimulationWindow.toogleAI) {
			return;
		}

		//initialize.
		if (is_init) {
			speedUp();
			Point dronePoint = drone.getOpticalSensorLocation();
			init_point = new Point(dronePoint);
			initPair = new Pair<Point, Integer>(dronePoint, 0);
			points.add(initPair);
			mGraph.addVertex(dronePoint);
			myGraph.addVertexAndEdge(initPair);
			is_init = false;
		}

		//count the real time of the running thread. every 360 loops of the thread is one second in real time.
		countTime++;
		if(countTime == 360) 
		{
			realTime++;
			pointWeight++;
			countTime=0;
		}



		Point dronePoint = drone.getOpticalSensorLocation();

		//if the battery of the drone is at 50% (150 seconds), or the return home button clicked.
		if (SimulationWindow.return_home || realTime>=150) {
			returnHome=true;
			stopAI = true;
			if(!myGraph.isEmpty()) {
				//calculate the angle between the drone and the last vertex of the graph.
				angle = angleBetweenPoints( myGraph.getLastPoint(),dronePoint);
				//the first step back home.
				if(turnAround) {
					slowDrasticFront();
					double spin = angle - drone.getRotation();
					if(spin < 0) {
						spin = (drone.getRotation()-angle)*-1;
					}
					spinBy(spin, true);
					turnAround = false;
					
				}
				else {
					if(degrees_left.isEmpty()) {
						speedUpHome();
					}
					//if the drone is at most 20 cm from the last vertex and there is no turns left to do, delete the last vertex and calculate the angle to the next vertex.
					if(Tools.getDistanceBetweenPoints(myGraph.getLastPoint(), dronePoint) <= 20 && degrees_left.isEmpty()) {
						myGraph.removeLastEdge();
						myGraph.removeLastVertex();
						myGraph.removeLastPoint();
						removeLastPoint();

						if(!myGraph.isEmpty()) {
							angle = angleBetweenPoints( myGraph.getLastPoint(),dronePoint);
							double spin = angle - drone.getRotation();
							if(spin < 0) {
								spin = (drone.getRotation()-angle)*-1;
							}
							//speedDown();
							slowDrasticFront();
							spinBy(spin, true);

						}
					}
				}				
				
			} else {
				drone.stop();
				ai_cpu.stop();
				drone.stop(deltaTime);
			}
			


		}
		else {
			//if the distance between the current point and the last point is at least 50 or the drone took a hard turn, adds a vertex to the graph.
			if (Tools.getDistanceBetweenPoints(getLastPoint(), dronePoint) >= max_distance_between_points || addPoint) {
				if(pointWeight!=0) {
					Pair<Point, Integer> pair = new Pair<Point, Integer>(dronePoint, pointWeight);
					points.add(pair);
					mGraph.addVertex(dronePoint);
					myGraph.addVertexAndEdge(pair);
					pointWeight = 0;
				}
				addPoint=false;

			}
		}
		if(!stopAI) {
			//not risky state
			if (!is_risky) {
				//if there is no turns left to do, speed up.
				if (shouldSpeedUp && degrees_left.size() == 0) {
					speedUp();
					shouldSpeedUp = false;
				}
				Lidar lidar = drone.lidars.get(0);
				if (lidar.current_distance <= max_risky_distance) {
					is_risky = true;
					risky_dis = lidar.current_distance;
					isFrontRisky = true;
				}

				Lidar lidar1 = drone.lidars.get(1);
				if (lidar1.current_distance <= 45) {
					is_risky = true;
					isRightRisky = true;
				}

				Lidar lidar2 = drone.lidars.get(2);
				if (lidar2.current_distance <= 45) {
					is_risky = true;
					isLeftRisky = true;
				}

				// this is when the drone took the right or left and the the opposite direction
				// opens up.
				if (degrees_left.size() == 0 && oppositeDirectionTurn) {
					Pair<Point, String> p = riskPoints.get(riskPoints.size() - 1);
					if (p.getValue().equals("left") && lidar2.current_distance > 200) {
						spinBy(-90, true);
					} else if (p.getValue().equals("right") && lidar1.current_distance > 200) {
						spinBy(90, true);

					}
					oppositeDirectionTurn = false;
				} else if (lidar1.current_distance > 270 && lastRightLidarDist < 100) {
					spinBy(45, true);
					if(!returnHome) {
						addPoint=true;
					}

				} else if (lidar2.current_distance > 270 && lastLeftLidarDist < 100) {
					spinBy(-45, true);
					if(!returnHome) {
						addPoint=true;
					}
				}

				lastRightLidarDist = lidar1.current_distance;
				lastLeftLidarDist = lidar2.current_distance;

			} else { // risky state
				oppositeDirectionTurn=false;
				Lidar lidar = drone.lidars.get(0);
				double frontLidarDist = lidar.current_distance;
				Lidar lidar1 = drone.lidars.get(1);
				double rightLidarDist = lidar1.current_distance;

				Lidar lidar2 = drone.lidars.get(2);
				double leftLidarDist = lidar2.current_distance;


				// only front is risky.
				if (isFrontRisky && degrees_left.size() == 0) {

					if(frontLidarDist < 80) {
						slowDrasticFront();
						sideLidarsDiff = lidar1.current_distance - lidar2.current_distance;
						if(sideLidarsDiff < 0) {
							spin_by = -60;
						} else {
							spin_by = 60;
						}
					} else {
						speedDown();
						sideLidarsDiff = lidar1.current_distance - lidar2.current_distance;
						if(sideLidarsDiff < 0) {
							spin_by = -20;
						} else {
							spin_by = 20;
						}
					}
					Pair<Point, String> p = new Pair<Point, String>(dronePoint, "front");
					riskPoints.add(p);
					shouldSpeedUp = true;
					spinBy(spin_by, true);
				}


				// only right risky.
				else if (isRightRisky && !isLeftRisky && degrees_left.size() == 0) {
					slowDownSides();
					if (rightLidarDist<35) {
						spinBy(-6, true);
						spinBy(2);

					}
					else { //right lidar > 45 && <85
						spinBy(-2, true);
						spinBy(1);

					}
					Pair<Point, String> p = new Pair<Point, String>(dronePoint, "right");
					riskPoints.add(p);
					shouldSpeedUp = true;
					oppositeDirectionTurn = true;
				}
				// only left risky.
				else if (!isRightRisky && isLeftRisky && degrees_left.size() == 0) {
					slowDownSides();
					if (leftLidarDist<35) {
						spinBy(6, true);
						spinBy(-2);


					}
					else { //right lidar > 45 && <85
						spinBy(2, true);
						spinBy(-1);

					}
					Pair<Point, String> p = new Pair<Point, String>(dronePoint, "left");
					riskPoints.add(p);
					shouldSpeedUp = true;
					oppositeDirectionTurn = true;
				}

				is_risky = false;
				isFrontRisky = false;
				isRightRisky = false;
				isLeftRisky = false;
			}
		}

	}

	int counter = 0;

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
			return initPair.getKey();
		}

		Point p1 = points.get(points.size() - 1).getKey();
		return p1;
	}

	public void removeLastPoint() {
		if (!points.isEmpty()) {
			points.remove(points.size() - 1);
		}
	}

	public Point getAvgLastPoint() {
		if (points.size() < 2) {
			return init_point;
		}

		Point p1 = points.get(points.size() - 1).getKey();
		Point p2 = points.get(points.size() - 2).getKey();
		return new Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
	}

	public double angleBetweenPoints(Point p, Point q) {
		double ans = 0;
		double deltaX = p.x - q.x;
		double deltaY = p.y - q.y;
		ans = Math.toDegrees(Math.atan2(deltaY, deltaX));
		if(ans < 0){
			ans += 360;
	    } 
		return ans;
	}

}
