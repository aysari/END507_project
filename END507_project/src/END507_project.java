
import java.io.*;
import java.util.*;

import ilog.concert.*;
import ilog.cplex.*;

public class END507_project{
	
	public static int M = 999;
	private static final double COOLING_RATE = 0.003;
	private static final double INITIAL_TEMP = 10000;
	private static final int MAX_ITERATION = 1200;
	private static stopwatchCPU timer1 = new stopwatchCPU();

	public static void main(String[] args) throws IOException{
		
		//initialization of output file to monitor results
		String file = "output.txt";
		PrintWriter outFile = new PrintWriter(file);
		
		//problem size is determined according to variables below
		int numberOfIncidents = 2;
		int numberOfVehicles = 2;
		int numberOfPotentialLocations = 10;
		
		//declaration of distance matrix
		double[][] distanceMatrix = new double[numberOfIncidents+numberOfPotentialLocations][numberOfIncidents+numberOfPotentialLocations];
		
		int numberOfnodes = numberOfIncidents+numberOfPotentialLocations;
		
		//declaration of d_ia matrix
		double[][] timeTravelToArc = new double[numberOfIncidents][numberOfnodes*(numberOfnodes-1)/2];
		
		generateDistanceMatrix(distanceMatrix, timeTravelToArc, numberOfIncidents);
		
		int[] incidentLocations = new int[numberOfIncidents];
		generateIncidents(incidentLocations);
		
		double[] occurenceTime = new double[numberOfIncidents];
		double[] serviceTime = new double[numberOfIncidents];
		generateIncidentParameters(incidentLocations, occurenceTime, serviceTime);
		
		int[] vehicles = new int[numberOfVehicles];
		generateVehicles(vehicles);
		
		/* Following variable determines the copy of initial locations. This is required because each vehicle may
		 * visit depot node more than one depending on the number of incidents and number of vehicles.
		 * */
		int numberOfCycle = 2;
		int[] initialLocations = new int[numberOfVehicles*numberOfCycle];
		generateInitialLocations(initialLocations, numberOfIncidents);
		
		outFile.print("iteration\t"+"incumbet\t"+"CPU time\t"+"no upt it");
		outFile.println();
		
		for(int i=0;i<30;i++) {
			ArrayList<Integer> waitingLocations = new ArrayList<>();
			findInitialSolution(waitingLocations,numberOfPotentialLocations,numberOfVehicles);
			ArrayList<Integer> currentLocationList = locationList(waitingLocations);
			
			int[] initialSolution = new int[numberOfVehicles];
			initialSolution = solutionArray(waitingLocations);
			
			int[] currentSolution = new int[numberOfVehicles];
			currentSolution = solutionArray(waitingLocations);
			
			int[] bestSolution = new int[numberOfVehicles];
			bestSolution = solutionArray(waitingLocations);
	
			double[][] updatedDistanceMatrix = new double[numberOfIncidents+numberOfVehicles*numberOfCycle][numberOfIncidents+numberOfVehicles*numberOfCycle];
			updatedDistanceMatrix = updateDistanceMatrix(updatedDistanceMatrix,distanceMatrix,waitingLocations,numberOfIncidents,numberOfCycle);
			
			int[][] beta = new int[incidentLocations.length][numberOfVehicles];
			generateBeta(beta);
			
			int[][] teta = new int[initialLocations.length+numberOfIncidents][numberOfVehicles];
			generateTeta(teta, numberOfCycle, numberOfIncidents);
			
			int cnt = 0;
			
			timer1 = new stopwatchCPU();
			
			double time1 = timer1.elapsedTime();
			
			double currentEnergy = routingModel(incidentLocations, vehicles, initialLocations, occurenceTime, serviceTime, updatedDistanceMatrix, timeTravelToArc, beta, teta);
			double best = currentEnergy;
			double neighbourEnergy=0;
			
			double temprature = INITIAL_TEMP;
			
			ArrayList<String> visitedList = new ArrayList<>();
			
			int cnt1 = 0;
			
			while(temprature>1&&cnt<MAX_ITERATION) {
				
				ArrayList<Integer> neighbourLocations = new ArrayList<>();
				int newLoc = 0;
				neighbourLocations = findNeighbour(currentLocationList,numberOfPotentialLocations,newLoc);
				String solutionString = getSolutionString(neighbourLocations);
				
				
				if(!visitedList.contains(solutionString)) {
					visitedList.add(solutionString);
					double[][] neighbourDistanceMatrix = new double[numberOfIncidents+numberOfVehicles*numberOfCycle][numberOfIncidents+numberOfVehicles*numberOfCycle];
					neighbourDistanceMatrix = updateDistanceMatrix(updatedDistanceMatrix,distanceMatrix,neighbourLocations,numberOfIncidents,numberOfCycle);
					neighbourEnergy = routingModel(incidentLocations, vehicles, initialLocations, occurenceTime, serviceTime, neighbourDistanceMatrix, timeTravelToArc, beta, teta);
					
					if(acceptanceProb(currentEnergy,neighbourEnergy,temprature)>Math.random()) {
						currentEnergy = neighbourEnergy;
						currentSolution = solutionArray(neighbourLocations);
						currentLocationList = locationList(neighbourLocations);
					}
					
					if(currentEnergy<best) {
						currentEnergy = neighbourEnergy;
						currentSolution = solutionArray(neighbourLocations);
						currentLocationList = locationList(neighbourLocations);
						bestSolution = solutionArray(currentLocationList);
						best = currentEnergy;
						cnt = 0;
					}
				}
				
				temprature *=1-COOLING_RATE;
				cnt++;
				cnt1++;
			}	
				outFile.print(cnt1+"\t"+Math.round(best)+"\t");
				time1 = timer1.elapsedTime();
				outFile.print(Math.round(time1)+"\t"+cnt+"\t");
				outFile.println();
				
				System.out.print(cnt1+"\t"+Math.round(best)+"\t");
				System.out.print(Math.round(time1)+"\t"+cnt+"\t");
				System.out.println();
		}
		
	
		outFile.close();
	}
	/*Incidents nodes are always the first n nodes, n is the number of incidents
	 * */
	public static void generateIncidents(int[] incidentLocations) {
		
		for(int i=0;i<incidentLocations.length;i++) {
			incidentLocations[i] = i+1;
		}
	}
	
	/*Following function generates the vehicles from 1 to k,
	 *k is the number of vehicles.  
	 * */
	public static void generateVehicles(int[] vehicles) {
		
		for(int i=0;i<vehicles.length;i++){
			vehicles[i]=i+1;
		}
	}
	
	/*
	 * */
	public static void generateInitialLocations(int[] initialLocations, int numberOfIncidents) {
		
		for(int i=0;i<initialLocations.length;i++){
			initialLocations[i]=numberOfIncidents+i+1;;
		}
	}
	/*Initial waiting locations of vehicles is determined randomly in the below function.
	 * */
	public static void findInitialSolution(ArrayList<Integer> waitingLocations, int numberOfPotentialLocations, int numberOfVehicles) {
		
		Random rand = new Random();
		
		for(int k=0;k<numberOfVehicles;k++) {
			boolean isAdded = true;
			
			while(isAdded) {
				int location = rand.nextInt(numberOfPotentialLocations)+1;
				if(!waitingLocations.contains(location)) {
					waitingLocations.add(location);
					isAdded = false;
				}
			}
		}
	}
	/* Incident parameters are randomly assigned in the below function according to the values mentioned in
	 * the project report.
	 * */
	public static void generateIncidentParameters(int[] incidentLocations, double[] occurenceTime, double[] serviceTime) {
		
		Random rand = new Random();
		
		for(int i:incidentLocations) {
			occurenceTime[i-1]=rand.nextInt(30)+1;
			serviceTime[i-1] = rand.nextInt(15)+1;
		}
	}
	/* Generation of distance matrix, firstly it generates coordinates of nodes
	 * and then it calculates euclidean distance between nodes. Distances of incidents to arcs
	 * are also calculated within this function. It is assumed that each incident is occured in the middle of
	 * the arc.
	 * */
	public static void generateDistanceMatrix(double[][] distanceMatrix,double[][] timeTravelToArc, int numberOfIncidents) {
		
		int[][] coordinatArr = new int[distanceMatrix.length][2];
		
		Random rand = new Random();
		
		for(int x = 0;x<distanceMatrix.length;x++) {
			for(int y = 0;y<2;y++) {
				coordinatArr[x][y] = rand.nextInt(50)+1;
			}
		}
		
		for(int from = 0;from<distanceMatrix.length;from++) {
			for(int to = 0;to<distanceMatrix.length;to++) {
				if(from==to)
					distanceMatrix[from][to]=0;
				else {
					distanceMatrix[from][to] = Math.hypot(coordinatArr[to][0]-coordinatArr[from][0], coordinatArr[to][1]-coordinatArr[from][1]);
				}
			}
		}
		
		ArrayList<Integer> arcs = new ArrayList<>();
		
		for(int i=0;i<numberOfIncidents;i++) {
			int arc = 0;
			for(int from=0;from<distanceMatrix.length;from++) {
				for(int to=from+1;to<distanceMatrix.length;to++) {
					arcs.add((from+1)*100+to+1);
					if(i==from) 
						timeTravelToArc[i][arc]= distanceMatrix[from][to]/2;
					else if(i==to)
						timeTravelToArc[i][arc]= distanceMatrix[from][to]/2;
					else {
						timeTravelToArc[i][arc]= ((distanceMatrix[i][arcs.get(arc)/100-1]<distanceMatrix[i][arcs.get(arc)%100-1])?distanceMatrix[i][arcs.get(arc)/100-1]:distanceMatrix[i][arcs.get(arc)%100-1])+distanceMatrix[from][to]/2;
					}
					arc++;
				}
			}
			arcs.clear();
		}
	}
	
	/* According to distance matrix and initial locations of the vehicles, t_ij matrix is updated. This function also
	 * includes the copy of initial locations since each vehicle may visit depot node more than once.
	 * */
	public static double[][] updateDistanceMatrix(double[][] updatedDistanceMatrix, double[][] distanceMatrix, ArrayList<Integer> isLocated, int numberOfIncidents, int cycle) {
		for(int from=0;from<numberOfIncidents;from++) {
			for(int to=from;to<numberOfIncidents;to++) {
				updatedDistanceMatrix[from][to]=distanceMatrix[from][to];
				updatedDistanceMatrix[to][from]=distanceMatrix[to][from];
			}
		}
		
		for(int from=0;from<updatedDistanceMatrix.length;from++) {
			for(int to=from;to<updatedDistanceMatrix.length;to++) {
				if(((from>=numberOfIncidents)||(to>=numberOfIncidents))&&((from<numberOfIncidents)||(to<numberOfIncidents))) {
					for(int c=0;c<cycle;c++) {
						updatedDistanceMatrix[from][to]=distanceMatrix[(from<numberOfIncidents?from:numberOfIncidents+isLocated.get((from-numberOfIncidents)/cycle)-1)][(to<numberOfIncidents?to:numberOfIncidents+isLocated.get((to-numberOfIncidents)/cycle)-1)];
						updatedDistanceMatrix[to][from]=distanceMatrix[(to<numberOfIncidents?to:numberOfIncidents+isLocated.get((to-numberOfIncidents)/cycle)-1)][(from<numberOfIncidents?from:numberOfIncidents+isLocated.get((from-numberOfIncidents)/cycle)-1)];
						to++;
					}
					to--;
				}
			}
			
		}
		for(int from=numberOfIncidents;from<updatedDistanceMatrix.length;from++) {
			for(int to=numberOfIncidents;to<updatedDistanceMatrix.length;to++) {
				if((from>=numberOfIncidents)&&(to>=numberOfIncidents)) {
					for(int c=0;c<cycle;c++) {
						updatedDistanceMatrix[from][to]=distanceMatrix[numberOfIncidents+isLocated.get((from-numberOfIncidents)/cycle)-1][numberOfIncidents+isLocated.get((to-numberOfIncidents)/cycle)-1];
						to++;
					}
					to--;
				}
			}
		}
		
		return updatedDistanceMatrix;
	}
	/* beta matrix is initialized in the below function. beta_ij gets 1 in each 2 elements in the array. By doing
	 * this some vehicles cannot response each incident.
	 * */
	public static void generateBeta(int[][] beta) {
		
		int cnt = 1;
		
		for(int i=0;i<beta.length;i++) {
			for(int k=0;k<beta[0].length;k++) {
				if(cnt%2==0) 
					beta[i][k]=1;
				else
					beta[i][k]=0;
				cnt++;
			}
		}
	}
	
	public static void generateTeta(int[][] teta, int cycle, int numberOfIncidents) {
		
		for(int k=0;k<teta[0].length;k++) {
			for(int i=0;i<teta.length;i++) {
				teta[i][k]=0;
			}
		}
		
		int cnt = numberOfIncidents;
		
		for(int k=0;k<teta[0].length;k++) {
			for(int i=0;i<cycle;i++) {
				teta[cnt][k]=1;
				cnt++;	
			}
		}
	}
	
	public static int[] solutionArray(ArrayList<Integer> isLocated) {
		int[] solArr = new int[isLocated.size()];
		
		for(int k=0;k<isLocated.size();k++) {
			solArr[k]=isLocated.get(k);
		}
		
		return solArr;
	}
	/**/
	public static ArrayList<Integer> locationList(ArrayList<Integer> isLocated) {
		ArrayList<Integer> locationList = new ArrayList<>();
		
		for(int k=0;k<isLocated.size();k++) {
			locationList.add(isLocated.get(k));
		}
		
		return locationList;
	}
	/* Following function finds a neighbour solution. It randomly generates an index and
	 * relocates the vehicles waiting in that locations to another random location.
	 * */
	public static ArrayList<Integer> findNeighbour(ArrayList<Integer> waitingLocations, int numberOfPotentialLocations, int newLoc) {
		
		Random rand = new Random();
		
		boolean isLocated = false;
		
		ArrayList<Integer> newWaitingLocations = locationList(waitingLocations);
		
		while(!isLocated) {
			int vehicle = rand.nextInt(newWaitingLocations.size());
			newLoc = rand.nextInt(numberOfPotentialLocations)+1;
			if((!newWaitingLocations.contains(newLoc))) {
				newWaitingLocations.set(vehicle, newLoc);
				isLocated = true;
			}			
		}
		
		return newWaitingLocations;
	}
	
	public static String getSolutionString(ArrayList<Integer> neighbour) {
		String str = "";
		
		for(int i=0;i<neighbour.size();i++) {
			str+=neighbour.get(i)+" ";
		}
		
		return str;
	}
	
	public static double acceptanceProb(double energy, double newEnergy, double temprature) {
		
		if(newEnergy<energy) {
			return 1.0;
		}
		
		return Math.exp((energy-newEnergy)/temprature);
	}
	
	public static double routingModel(int[] incidentLocations, int[] vehicles, int[] initialLocations, double[] occurenceTime, double[] serviceTime, 
									double[][] travelTimeBtwNode, double[][] travelTimeToArc, int[][] beta, int[][] teta) {
		
		double objVal=1000000;
		//sets
		int numberOfIncidents = incidentLocations.length;
		
		int numberOfInitialLocations = initialLocations.length;		
		
		int numberOfNodes = numberOfIncidents+numberOfInitialLocations;
		int[] allNodes = new int[numberOfNodes];
		System.arraycopy(incidentLocations,0,allNodes,0,incidentLocations.length);
		System.arraycopy(initialLocations,0,allNodes,incidentLocations.length,initialLocations.length);
		Arrays.sort(allNodes);

		int numberOfVehicles = vehicles.length;
		int numberOfArcs = travelTimeToArc[0].length;
		
		try {
			IloCplex cplex = new IloCplex();
			
			/* In order to reduce time required to run the code, output of mathematical model is restricted.
			 * following cplex.setOut(null); line can be deleted to observe the solving process of model.
			 * */

			cplex.setOut(null);
			
			//decision variables
			IloIntVar[][] y = new IloIntVar[numberOfNodes][numberOfVehicles];
			ArrayList<IloIntVar[][]> x = new ArrayList<IloIntVar[][]>();
			IloNumVar[][] arrivalTime = new IloNumVar[numberOfNodes][numberOfVehicles];
			IloNumVar[][] departureTime = new IloNumVar[numberOfNodes][numberOfVehicles];
			IloNumVar[] responseTime = new IloNumVar[numberOfNodes];
			IloNumVar[][] alpha = new IloNumVar[numberOfNodes][numberOfArcs];
			
			for(int k:vehicles) {
				x.add(new IloIntVar[numberOfNodes][numberOfNodes]);
				for(int i:allNodes) {
					for(int j:allNodes) {
						if(!(i==j)) {
							x.get(k-1)[i-1][j-1] = cplex.intVar(0, 1, "x_"+i+","+j+","+k);
						}
					}
				}
			}
			
			for(int i:allNodes) {
				responseTime[i-1] = cplex.numVar(0, Double.MAX_VALUE,"R_"+i);
				for(int k:vehicles) {
					y[i-1][k-1] = cplex.intVar(0, 1,"y_"+i+","+k);
				}
			}
			
			for(int i:allNodes) {
				for(int k:vehicles) {
					arrivalTime[i-1][k-1] = cplex.numVar(0, Double.MAX_VALUE,"A_"+i+k);
					departureTime[i-1][k-1] = cplex.numVar(0, Double.MAX_VALUE,"D_"+i+k);
				}
			}
			
			for(int a = 0; a<numberOfArcs; a++) {
				for(int i:incidentLocations) {
					alpha[i-1][a] = cplex.numVar(0, Double.MAX_VALUE,"alpha("+i+","+(a+1)+")");
				}
			}
			//end-decision variables
			
			//objective-------------------------------------------------------------
			IloLinearNumExpr obj = cplex.linearNumExpr();
			
			for(int a = 0; a<numberOfArcs; a++) {
				for(int i:incidentLocations) {
					obj.addTerm(alpha[i-1][a],1);
				}
			}
			
			cplex.addMinimize(obj);
			//end-objective-----------------------------------------------------------
			
			//constraints
			
			//constraint 2 
			
			for(int a = 0; a<numberOfArcs; a++) {
				for(int i:incidentLocations) {
					IloLinearNumExpr constraint2 = cplex.linearNumExpr();
					constraint2.addTerm(1, alpha[i-1][a]);
					cplex.addEq(constraint2, cplex.prod(cplex.diff(responseTime[i-1], occurenceTime[i-1]), 1/travelTimeToArc[i-1][a]));
				}
			}			
			
			//constraint 3
			
			for(int i:incidentLocations) {
				for(int k:vehicles) {
					IloLinearNumExpr constraint3 = cplex.linearNumExpr();
					constraint3.addTerm(1, y[i-1][k-1]);
					cplex.addLe(constraint3, beta[i-1][k-1]);
				}
			}
			
			//constraint 4
			
			for(int i:incidentLocations) {
				IloLinearNumExpr constraint4 = cplex.linearNumExpr();
				for(int k:vehicles) {
					constraint4.addTerm(1, y[i-1][k-1]);
				}
				cplex.addLe(constraint4, 1);
			}
			
			//constraint 5 & 6
			
			for(int k:vehicles) {
				for(int i:initialLocations) {
					IloLinearNumExpr constraint5 = cplex.linearNumExpr();
					IloLinearNumExpr constraint6 = cplex.linearNumExpr();
					for(int j:allNodes) {
						if(!(i==j)) {
							constraint5.addTerm(1, x.get(k-1)[i-1][j-1]);
							constraint6.addTerm(1, x.get(k-1)[j-1][i-1]);
						}
					}
					cplex.addLe(constraint5, M*teta[i-1][k-1]);
					cplex.addLe(constraint6, M*teta[i-1][k-1]);
				}
			}
			
			//constraint 7
			
			for(int k:vehicles) {
				for(int i:incidentLocations) {
					IloLinearNumExpr constraint7_1 = cplex.linearNumExpr();
					IloLinearNumExpr constraint7_2 = cplex.linearNumExpr();
					for(int j:allNodes) {
						if(!(i==j)) {
							constraint7_1.addTerm(1, x.get(k-1)[i-1][j-1]);
							constraint7_2.addTerm(1, x.get(k-1)[j-1][i-1]);
						}
					}
					cplex.addLe(constraint7_1, constraint7_2);
				}
			}
			
			//constraint 8 & 9
			
			for(int k:vehicles) {
				for(int i:incidentLocations) {
					IloLinearNumExpr constraint8 = cplex.linearNumExpr();
					IloLinearNumExpr constraint9 = cplex.linearNumExpr();
					for(int j:allNodes) {
						if(!(i==j)) {
							constraint8.addTerm(1, x.get(k-1)[i-1][j-1]);
							constraint9.addTerm(1, x.get(k-1)[j-1][i-1]);
						}
					}
					cplex.addEq(constraint8, y[i-1][k-1]);
					cplex.addEq(constraint9, y[i-1][k-1]);
				}
			}
			
			//constraint 10 & 11
			
			for(int k:vehicles) {
				for(int i:allNodes) {
					for(int j:allNodes) {
						if(!(i==j)) {
							IloLinearNumExpr constraint10 = cplex.linearNumExpr();
							IloLinearNumExpr constraint11 = cplex.linearNumExpr();
							
							constraint10.addTerm(1, departureTime[i-1][k-1]);
							constraint10.addTerm(-1, arrivalTime[j-1][k-1]);
							constraint10.addTerm(M, x.get(k-1)[i-1][j-1]);
							constraint11.addTerm(1, departureTime[i-1][k-1]);
							constraint11.addTerm(-1, arrivalTime[j-1][k-1]);
							constraint11.addTerm(-M, x.get(k-1)[i-1][j-1]);
							
							cplex.addLe(constraint10, -travelTimeBtwNode[i-1][j-1]+M);
							cplex.addGe(constraint11, -travelTimeBtwNode[i-1][j-1]-M);
						}
					}
				}
			}
			
			//constraint 12
			for(int i:incidentLocations) {
				for(int k:vehicles) {	
					IloLinearNumExpr constraint12 = cplex.linearNumExpr();
					constraint12.addTerm(1, departureTime[i-1][k-1]);
					constraint12.addTerm(-1, arrivalTime[i-1][k-1]);
					cplex.addGe(constraint12, 0);
				}
			}
			
			//constraint 13
			for(int k:vehicles) {
				for(int i:incidentLocations) {	
					IloLinearNumExpr constraint13 = cplex.linearNumExpr();
					constraint13.addTerm(1, departureTime[i-1][k-1]);
					constraint13.addTerm(-1, arrivalTime[i-1][k-1]);
					constraint13.addTerm(-serviceTime[i-1], y[i-1][k-1]);
					cplex.addEq(constraint13, 0);
				}
			}
			
			//constraint 14&15
			for(int i:incidentLocations) {
				for(int k:vehicles) {	
					IloLinearNumExpr constraint14 = cplex.linearNumExpr();
					IloLinearNumExpr constraint15 = cplex.linearNumExpr();
					constraint14.addTerm(1, arrivalTime[i-1][k-1]);
					constraint14.addTerm(-M, y[i-1][k-1]);
					constraint15.addTerm(1, departureTime[i-1][k-1]);
					constraint15.addTerm(-M, y[i-1][k-1]);
					cplex.addLe(constraint14, 0);
					cplex.addLe(constraint15, 0);
				}
			}
			
			//constraint 16
			for(int k:vehicles) {
				for(int i:incidentLocations) {
					for(int j:allNodes) {
						if(!(i==j)) {
							IloLinearNumExpr constraint16 = cplex.linearNumExpr();
							constraint16.addTerm(occurenceTime[i-1], y[i-1][k-1]);
							constraint16.addTerm(travelTimeBtwNode[i-1][j-1], x.get(k-1)[i-1][j-1]);
							constraint16.addTerm(-1, arrivalTime[i-1][k-1]);
							cplex.addLe(constraint16, 0);
						}
					}
				}
			}
			
			//constraint 17
			for(int i:incidentLocations) {
				for(int k:vehicles) {	
					IloLinearNumExpr constraint17 = cplex.linearNumExpr();
					constraint17.addTerm(1, responseTime[i-1]);
					constraint17.addTerm(-1, arrivalTime[i-1][k-1]);
					cplex.addGe(constraint17, 0);
				}
			}
			
			//constraint 18
			for(int i:incidentLocations) {
				IloLinearNumExpr constraint18 = cplex.linearNumExpr();
				for(int k:vehicles) {					
					constraint18.addTerm(1, y[i-1][k-1]);
				}
				constraint18.addTerm(1/M, responseTime[i-1]);
				cplex.addGe(constraint18, 1);
			}
			
			//end-constraints
			
			//printing open form of MIP model
			cplex.exportModel("END507_projectModel_openForm.lp");
			
			//solve
			
			/* In this section MIP is solved, the values of decision variables are commented below in order,
			 * to reduce run time of the code. The comment on these lines can be deleted to observe the values
			 * of decision variables.
			 * */
			if(cplex.solve()) {
				objVal = cplex.getObjValue();
				
				for(int k:vehicles) {
					for(int i:allNodes) {
						//System.out.println("A_"+i+"\t"+Math.round(cplex.getValue(arrivalTime[i-1][k-1])));
						//System.out.println("D_"+i+"\t"+Math.round(cplex.getValue(departureTime[i-1][k-1])));
						for(int j:allNodes) {
							if(!(i==j)) {
//								System.out.println("x" + i +""+j+""+ k +": "+cplex.getValue(x.get(k-1)[i-1][j-1]));
							}
						}
					}
				}
//				for(int i:incidentLocations) {
//					for(int k:vehicles) {
//						System.out.println("y" + i +","+ k +"\t"+Math.round(cplex.getValue(y[i-1][k-1])));
//					}
//					
//					System.out.println("R_"+i+"\t"+Math.round(cplex.getValue(responseTime[i-1])));
//				}
			}
			else {
				System.out.println("Model not solved!");
			}
			
			cplex.close();
		}
		
		catch(IloException exc) {
			System.out.println(exc+" error");
		}
		
		return objVal;
	}
}
