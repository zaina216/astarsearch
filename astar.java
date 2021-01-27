//**********************************************************************************************************
// This code has been designed to calculate the shortest route between two user chosen points on a grid.
// Written by Zain A. (September 2020) 
//**********************************************************************************************************

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.Rectangle2D;
import java.util.*;
import java.util.List;
import javax.swing.*;
import javax.swing.Timer;
import java.awt.event.KeyEvent;

//Constructor defines and creates the window
public class astar extends JFrame{
    public static final Color DEFAULT_COLOR = Color.DARK_GRAY;
    static JButton buttonStart;
    static int spaceBetweenSquares = 2;

    static node[][] shapesAry = new node[16][9];
    // Fills the newly created shapesAry pathfinding grid with node types to be later manipulated
    public astar( node[][] shapesAry) throws InterruptedException {
        astar.shapesAry = shapesAry;
        for (int x = 0; x < 16; x++) {
            for (int y = 0; y < 9; y++) {
                shapesAry[x][y] = new node(new Rectangle2D.Double(spaceBetweenSquares + x*80, spaceBetweenSquares + y*80,
                        80 - spaceBetweenSquares*2, 80 - spaceBetweenSquares*2), DEFAULT_COLOR, x, y);

            }
        }

        this.setTitle("Shapes");
        //jpa is added to the right of the screen
        JPanel jpa = new JPanel();
        buttonStart = new JButton("Start");
        buttonStart.setVisible(true);
       
        // Box allows buttons to be stacked on top of each other
        Box box1 = Box.createVerticalBox();
        box1.add(buttonStart, Component.CENTER_ALIGNMENT);
        box1.add(Box.createVerticalStrut(11));
        
        // Sets attributes for the window for the displayed e.g. a size of 1280x720
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        update panel = new update(shapesAry);
        jpa.add(box1);
        jpa.setBackground(Color.BLACK);
        this.add(jpa);
        jpa.add(panel);
        this.setSize(1280, 720);
        this.setResizable(false);
        this.setLocationByPlatform(true);
        this.pack();
        this.getContentPane().setBackground(Color.BLACK);
        this.setVisible(true);

    }
    
    //node class creates a modular approach: colour can be changed, and can be set as an obstacle easily
    static class node {
        int[] position = {0, 0};
        double f=0;  int g=0;  double h=0;
        int[] offspring = {0, 0};
        private Shape shape;
        boolean partOfRoute = false;
        boolean alreadyVisited = false;
        private Color color = Color.DARK_GRAY;
        boolean isObstacle = false;

        public node(Shape shape, Color color, int x, int y) {
            super();
            this.shape = shape;
            this.color = color;
            position[0] = x;
            position[1] = y;
        }

        public Shape getShape() {
            return shape;
        }

        public Color getColor() {
            return color;
        }

        public void setColor(Color color) {
            this.color = color;


        }
    }

    static class update extends JPanel{
        //Finds the neighbouring nodes for the current node. Takes into account nodes at the edge of the grid
        public static int[][] findNeighbouringNodes(node currentNode, int xEnd, int yEnd){
            int[] position = currentNode.position;
            int[] top = {position[0], position[1]-1};
            int[] bottom = {position[0], position[1]+1};

            int[] right = {position[0]+1, position[1]};
            int[] bottom_right = {position[0]+1, position[1]+1};
            int[] top_right = {position[0]+1, position[1]-1};

            int[] left = {position[0]-1, position[1]};
            int[] bottom_left = {position[0]-1, position[1]+1};
            int[] top_left = {position[0]-1, position[1]-1};

            //four corners
            if (Arrays.equals(currentNode.position, new int[]{0, 0})) {
                return new int[][] {{0, 1}, {1, 0}, {1, 1}};
            } else if (Arrays.equals(currentNode.position, new int[]{0, yEnd})){
                return new int[][] {{0, yEnd-1}, {1, yEnd-1}, {1, yEnd}};
            } else if (Arrays.equals(currentNode.position, new int[]{xEnd, 0})) {
                return new int[][]{{xEnd-1, 0}, {xEnd-1, 1}, {xEnd, 1}};
            } else if (Arrays.equals(currentNode.position, new int[]{xEnd, yEnd})) {
                return new int[][]{{xEnd - 1, yEnd}, {xEnd - 1, yEnd - 1}, {xEnd, yEnd - 1}};
            }
            else if (Arrays.equals(currentNode.position, new int[]{1, 0})){
                return new int[][]{{0, 0}, {0, 1}, {1, 1}, {2, 1}, {2, 0}};
            //sides
            }
            else if(currentNode.position[0] - 1 < 0) {
                return new int[][] {top, top_right, right, bottom_right, bottom};
            } else if(currentNode.position[0] + 1 > 15) {
                return new int[][] {top, top_left, left, bottom_left, bottom};
            } else if(currentNode.position[1] - 1 < 0) {
                return new int[][] {left, bottom_left, bottom, bottom_right, right};
            } else if(currentNode.position[1] + 1 > 8) {
                return new int[][] {left, top_left, top, top_right, right};
            }
            //rest of squares
            return new int[][] {left, top_left, top, top_right, right, bottom_left, bottom, bottom_right};

        }


        int[] startPt = new int[]{0, 0};
        int[] endingPt = new int[]{10, 3};
        node[][] searchSpace = new node[9][16];
        private List<node> shapes;
        int x = 15; int y = 8;
        private Random rand = new Random();
        boolean startButtonClicked = false;

        //subroutine performing A* search itself
        public void astarsearch(ArrayList<node> openList, ArrayList<node> closedList,node[] currentNode,
                                ArrayList<node> finalRoute, ArrayList<node> borders, node endNode) {

            searchSpace[startPt[0]][startPt[1]].offspring = new int[]{startPt[0], startPt[1]};
            openList.add(searchSpace[startPt[0]][startPt[1]]);
            searchSpace[startPt[0]][startPt[1]].setColor(Color.GREEN);
            searchSpace[endingPt[0]][endingPt[1]].setColor(Color.RED);
            int loopCount = 0;
            
            foundRoute:
            while (openList.size() != 0 && (loopCount < 5000)) {
                int idxLeastF = findMinFScore(openList);

                //pops current node off the open list and adds it to the closed list
                currentNode[0] = openList.remove(idxLeastF);
                closedList.add(currentNode[0]);

                //get all neighbouring nodes of current node and put them in a list
                int[][] neighbourPositions = findNeighbouringNodes(currentNode[0], 15, 8);
                ArrayList<node> neighbourNodes2 = new ArrayList<>();
                loopCount++;
                for (int[] neighbourPosition : neighbourPositions) {
                    if (!searchSpace[neighbourPosition[0]][neighbourPosition[1]].isObstacle) {
                        neighbourNodes2.add(searchSpace[neighbourPosition[0]][neighbourPosition[1]]);
                        borders.add(searchSpace[neighbourPosition[0]][neighbourPosition[1]]);
                        borders = removeDupes(borders);
                    }
                }

                node[] neighbourNodes = new node[neighbourNodes2.size()];
                for (int i = 0; i < neighbourNodes2.size(); i++) {
                    neighbourNodes[i] = neighbourNodes2.get(i);
                }

                //for each neighbour of current
                outerLoop:
                for (int x = 0; x < neighbourNodes.length; x++) {
                    //ending condition
                    if (Arrays.equals(neighbourNodes[x].position, endingPt)) {
                        currentNode[0].partOfRoute = true;

                        //backtrack here
                        currentNode[0] = neighbourNodes[x];
                        System.out.println("in the finishing condition");
                        finalRoute = backtrack(searchSpace, searchSpace[startPt[0]][startPt[1]], neighbourNodes[x], closedList);
                        System.out.println("after backtrack");
                        System.out.println("Finished.");
                        //Breaks out of the main while loop.
                        break foundRoute;
                    }

                    for (node node : closedList) {
                        if(Arrays.equals(neighbourNodes[x].position, node.position)){
                            continue outerLoop;
                        }
                    }
                    
                    
                    neighbourNodes[x].g = currentNode[0].g + 1;
                    neighbourNodes[x].h = calculateH(neighbourNodes[x], endNode);
                    neighbourNodes[x].f = neighbourNodes[x].g + neighbourNodes[x].h;

                    for (node node : openList) {
                        if(Arrays.equals(neighbourNodes[x].position, node.position)){
                            if(neighbourNodes[x].g > node.g){
                                continue outerLoop;
                            }
                        }
                    }

                    currentNode[0].partOfRoute = true;
                    searchSpace[currentNode[0].position[0]][currentNode[0].position[1]].partOfRoute = true;
                    finalRoute.add(currentNode[0]);

                    openList.add(neighbourNodes[x]);
                }
                //refereshes the window so colours are displayed correctly
                repaint();

            }

            //timers are a thread-safe way of delaying the display of colour in a window (or any other timed GUI delay)
            final boolean[] t2Start = {false};
            final Timer t = new Timer(20, null);
            ArrayList<node> finalBorders = borders;
            t.addActionListener(new ActionListener() {
                int x = 0;

                public void actionPerformed(ActionEvent e) {
                    searchSpace[finalBorders.get(x).position[0]][finalBorders.get(x).position[1]].setColor(Color.GREEN);
                    repaint();
                    if (x > finalBorders.size() - 2) {
                        t2Start[0] = true;
                        t.stop();
                    }
                    x++;
                }
            });
            t.setRepeats(true);
            t.start();

//            controlTimers(Color.GREEN, finalBorders, searchSpace);
//            t2Start[0] = true;

            final Timer t2 = new Timer(10,null);
            ArrayList<node> finalRoute1 = finalRoute;
            t2.addActionListener(new ActionListener(){
                int x=0;

                public void actionPerformed(ActionEvent e){
                    if(t2Start[0]) {
                          searchSpace[finalRoute1.get(x).position[0]][finalRoute1.get(x).position[1]].setColor(Color.blue);
                        repaint();
                        if (x > finalRoute1.size() - 2) {
                            t2.stop();
                        }
                        x++;
                    }
                }
            });
            t2.setRepeats(true);
            t2.start();

        }

        //updates items on screen
        //astarsearch here
        private static volatile boolean wPressed = false;

        private static volatile boolean sPressed = false;
        private int[] lastStart = new int[]{0, 0};
        private int[] lastEnd = new int[]{0, 1};
        public update(node[][] shapesAry){

            //senses if W and/or S are pressed on the keyboard. user can choose a start and end point
            KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyEventDispatcher() {
                @Override
                public boolean dispatchKeyEvent(KeyEvent ke) {
                    synchronized (update.class) {
                        switch (ke.getID()) {
                            case KeyEvent.KEY_PRESSED:
                                if (ke.getKeyCode() == KeyEvent.VK_W) {
                                    wPressed = true;
                                }
                                if (ke.getKeyCode() == KeyEvent.VK_S){
                                    sPressed = true;
                                }
                                break;
                            case KeyEvent.KEY_RELEASED:
                                if (ke.getKeyCode() == KeyEvent.VK_W) {
                                    wPressed = false;
                                }
                                if (ke.getKeyCode() == KeyEvent.VK_S){
                                    sPressed = true;
                                }
                                break;
                        }
                        return false;
                    }
                }
            });


            astar.buttonStart.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    startButtonClicked = true;
                }
            });
                this.searchSpace = shapesAry;
                System.out.println("x: "+shapesAry[15][8].position[0]+", y: "+shapesAry[15][8].position[1]);
                //Enables the user to drag across the screen to select obstacles in one go
                addMouseMotionListener(new MouseAdapter() {
                    @Override
                    public void mouseDragged(MouseEvent e) {
                        super.mouseDragged(e);
                        for (int i = 0; i < 16; i++) {
                            for (int j = 0; j < 9; j++) {
                                if (searchSpace[i][j].getShape().contains(e.getPoint())) {
                                    searchSpace[i][j].setColor(Color.PINK);
                                    searchSpace[i][j].isObstacle = true;
                                }
                            }
                        }
                        repaint();
                    }
                });
                addMouseListener(new MouseAdapter() {
                    @Override
                    public void mouseClicked(MouseEvent e) {

                        for (int i = 0; i < 16; i++) {
                            for (int j = 0; j < 9; j++) {
                                if (searchSpace[i][j].getShape().contains(e.getPoint())) {
                                    if (wPressed) {
                                        startPt = searchSpace[i][j].position;
                                        searchSpace[i][j].setColor(Color.GREEN);
                                        searchSpace[lastStart[0]][lastStart[1]].setColor(DEFAULT_COLOR);
                                        lastStart = new int[]{i, j};
                                    } else if (sPressed){
                                        endingPt = searchSpace[i][j].position;
                                        searchSpace[i][j].setColor(Color.RED);
                                        searchSpace[lastEnd[0]][lastEnd[1]].setColor(DEFAULT_COLOR);
                                        lastEnd = new int[]{i, j};
                                    } else {
                                        searchSpace[i][j].setColor(Color.PINK);
                                        searchSpace[i][j].isObstacle = true;
                                    }
                                }
                            }
                        }
                        repaint();
                    }

                });


                ArrayList<node> closedList = new ArrayList<>();
                ArrayList<node> borders = new ArrayList<>();
                ArrayList<node> finalRoute = new ArrayList<>();

                //getting the current starting node into the open list
                ArrayList<node> openList = new ArrayList<>();
                node endNode = searchSpace[endingPt[0]][endingPt[1]];

                final node[] currentNode = {searchSpace[startPt[0]][startPt[1]]};

                System.out.println(Arrays.toString(currentNode[0].position));
                searchSpace[startPt[0]][startPt[1]].setColor(Color.GREEN);
                searchSpace[endingPt[0]][endingPt[1]].setColor(Color.RED);


            astar.buttonStart.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    astarsearch(openList, closedList, currentNode,
                            finalRoute, borders, endNode);
                }
            });

        }


        public ArrayList<node> backtrack(node[][] searchSpace, node startNode, node endNode, ArrayList<node> finalRoute) {
            //go through partOfRoute and check for min g score. Then colour that in differently to the rest of the grid. This builds the final path to be displayed.

            node tempNode = endNode;
            System.out.println("endNode pos:"+Arrays.toString(startNode.position));
            System.out.println("endNode pos:"+Arrays.toString(endNode.position));
            ArrayList<node> chosenRoute = new ArrayList<>();
            int infiniteProtection = 0;
            while (!Arrays.equals(startNode.position, tempNode.position) && (infiniteProtection < 1000)) {
                //generate neighbour nodes
                int[][] neighbourPositions = findNeighbouringNodes(tempNode, 15, 8);
                node[] neighbourNodes = new node[neighbourPositions.length];
                ArrayList<node> candidatesForBestRoute = new ArrayList<>();

                for (int i = 0; i < neighbourNodes.length; i++) {
                    neighbourNodes[i] = searchSpace[neighbourPositions[i][0]][neighbourPositions[i][1]];
                }

                //end generation of neighbour nodes for the current iteration

                for (node neighbourNode : neighbourNodes) {
                    if (neighbourNode.partOfRoute) { // has already been nacktracked
                        if (!neighbourNode.alreadyVisited && finalRoute.contains(neighbourNode)) {
                            neighbourNode.alreadyVisited = true;
                            candidatesForBestRoute.add(neighbourNode);
                        }
                    }

                }
                int minGScore = findMinGScore(candidatesForBestRoute);
                for (astar.node node : candidatesForBestRoute) {
                    if (minGScore == node.g) {
                        System.out.println("min G: "+Arrays.toString(node.position));
                        tempNode = searchSpace[node.position[0]][node.position[1]];
                        chosenRoute.add(searchSpace[node.position[0]][node.position[1]]);
                        break;
                    }
                }

                infiniteProtection++;
            }
            System.out.println(infiniteProtection);
            for (astar.node node : chosenRoute) {
                System.out.print(Arrays.toString(node.position));
            }
            System.out.println();
            return chosenRoute;
        }

        //set automatically removes duplications from the ArrayList. This is to stop the A* algorithm from diverging from its path
        public ArrayList<node> removeDupes(ArrayList<node> borders) {
            Set<node> set = new LinkedHashSet<>(borders);
            borders.clear();
            borders.addAll(set);
            return borders;
        }

        public static int findMinFScore(ArrayList<node> openList){
            double minF = 90000;
            int idx = 0;
            for(int x = 0; x < openList.size(); x ++){
                if(openList.get(x).f < minF){
                    minF = openList.get(x).f;
                    idx = x;
                }
            }

            return idx;
        }

        public static int findMinGScore(ArrayList<node> openList){
            int minG = openList.get(0).g;
            for(int x = 0; x < openList.size(); x ++){
                if(openList.get(x).g < minG){
                    minG = openList.get(x).g;
                }
            }

            return minG;
        }

        //Euclidean distance between the current node and the end node
        public static double calculateH (node node1, node node2){
            return Math.sqrt(Math.pow(node1.position[0]-node2.position[0], 2) + Math.pow(node1.position[1]-node2.position[1], 2));
        }


        @Override
        //draws things onto screen initially
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            g.setColor(Color.BLACK);
            g.fillRect(0, 0, 1280, 720);

            Graphics2D gr2D = (Graphics2D) g.create();

            for (int i = 0; i < 16; i++) {
                for (int j = 0; j < 9; j++) {
                    gr2D.setColor(searchSpace[i][j].getColor());
                    gr2D.fill( searchSpace[i][j].getShape());
                }
            }
            gr2D.dispose();
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(1280, 720);
        }

    }
    
    // Main entry point to code. When run, all nodes are given a default config (Dark grey colour, not an obstacle, etc...)
    public static void main(String[] args){
        node[][] shapesAry = new node[16][9];
        for (int x = 0; x < 16; x++) {
            for (int y = 0; y < 9; y++) {
                shapesAry[x][y] = new node(new Rectangle2D.Double(spaceBetweenSquares + x*80, spaceBetweenSquares + y*80,
                        80 - spaceBetweenSquares*2, 80 - spaceBetweenSquares*2), DEFAULT_COLOR, x, y);

            }
        }

        SwingUtilities.invokeLater(new Runnable() {
            public void run() {
                try {
                    new astar(shapesAry);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
    }
}
