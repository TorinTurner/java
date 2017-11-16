package snake;
//package snake_game;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.RenderingHints;
import java.awt.Robot;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.InputEvent;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.util.Vector;

import javax.swing.ActionMap;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.KeyStroke;

//Each circle in snake body
class snake_node{
 int centerx, centery, radius, pcenterx, pcentery;
 snake_node(){
  
  radius = 10;
 }
}
//actual gui for game
public class Snake_game extends JFrame {
 snake panel = null;
 JMenuBar menu = null;
 JMenu controls = null;
 JMenuItem play = null;
 JMenuItem restart = null;
 JMenuItem instructions = null;
 boolean firstplay = true;
 Snake_game(){
  this.setResizable(false);
  menu = new JMenuBar();
  play = new JMenuItem("Play");
  controls = new JMenu("Controls");  
  restart = new JMenuItem("Restart");
  instructions = new JMenuItem("Instructions");
  restart.setMnemonic(KeyEvent.VK_R);
  restart.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_R, InputEvent.CTRL_MASK));
  restart.setMnemonic(KeyEvent.VK_I);
  instructions.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_I, InputEvent.CTRL_MASK));
  play.setMnemonic(KeyEvent.VK_P);
  play.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_P, InputEvent.CTRL_MASK));
  controls.add(play);
  controls.add(restart);
  controls.add(instructions);
  menu.add(controls);
  this.setJMenuBar(menu);
  //GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice().setFullScreenWindow(this);
  panel = new snake();
  this.setLayout(new BorderLayout());
  this.setMinimumSize(new Dimension(800, 595));
  panel.dim = this.getSize();
  this.add(panel, BorderLayout.CENTER);
  this.setLocationRelativeTo(null);
  this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
  play.addActionListener(new ActionListener(){public void actionPerformed(ActionEvent eee){play();}});
  restart.addActionListener(new ActionListener(){public void actionPerformed(ActionEvent ee){restart();}});
  instructions.addActionListener(new ActionListener(){public void actionPerformed(ActionEvent ee){instruct();}});
  this.addKeyListener(new KeyAdapter(){public void keyPressed(KeyEvent e){panel.keyPress(e);}});      
  this.setVisible(true);
  this.pack();
 }
        //function to start thread for starting game
 public void play(){
  if(firstplay){
   panel.t.start();
   firstplay = false;
   play.setEnabled(false);
  }
 }
        //function for restarting game, it will resize the snake to its initial position and initial length
 public void restart(){
  int pos = 0;
  panel.vector.clear();
  snake_node[] snode = new snake_node[5];
  for(int i =0; i < 5; i++){
   snode[i] = new snake_node();
   snode[i].centerx = 200;
   snode[i].centery = 150-pos;
   snode[i].pcentery = 150-pos;
   snode[i].pcenterx = 200;
   panel.vector.add(snode[i]);
   pos = pos+10;
  }
  panel.gameover = false;
  panel.dir = 'D';
  panel.repaint();
 }
        //function to be called on the click of instructions
 public void instruct(){
  JOptionPane.showMessageDialog(this, "1. Ctrl+R - Restart \n2. S - Increase Snake speed \n3. D - Decrease Snake speed \n4. LEFT - Turn left \n5. RIGHT - Turn right \n6. UP - go up \n7. DOWN - go down \n8. Ctrl+I - to view Instructions", "Instructions",1);
 }
 public static void main(String[] ar){
  new Snake_game();// object of the game class
 }
}

//a panel where all the drawings take place and it is embedded into JFrame
class snake extends JPanel implements Runnable{
 Vector<snake_node> vector = null;   //vector which will hold all the beads of snake body
 snake_node[] snode = null;
 int centx = 0, centy = 0;
 int inc = 10, pos = 0;
 char dir = 'D';
 Dimension dim = null;
 Thread t = null;
 boolean b = true, gameover = false;
 snake(){
  vector = new Vector();
  snode = new snake_node[5];
  for(int i =0; i < 5; i++){
   snode[i] = new snake_node();
   snode[i].centerx = 200;
   snode[i].centery = 150-pos;
   snode[i].pcentery = 150-pos;
   snode[i].pcenterx = 200;
   vector.add(snode[i]);
   pos = pos+10;
  }
  t = new Thread(this);
  this.setLayout(null);
  this.setBackground(Color.black);
  this.setDoubleBuffered(true);
  dim = this.getSize();
  this.addKeyListener(new KeyAdapter(){public void keyTyped(KeyEvent e){keyPress(e);}});
  this.setVisible(true);
 }
 int time = 70;
 boolean gamepause = true;
        //setting keyevents 
 void keyPress(KeyEvent e){
  if(e.getKeyCode() == KeyEvent.VK_DOWN){
   if(dir != 'U')
    dir = 'D';
  }
  else if(e.getKeyCode() == KeyEvent.VK_UP){
   if(dir != 'D')
    dir = 'U';
  }
  else if(e.getKeyCode() == KeyEvent.VK_LEFT){
   if(dir != 'R')
    dir = 'L';
  }
  else if(e.getKeyCode() == KeyEvent.VK_RIGHT){
   if(dir != 'L')
    dir = 'R';
  }
  else if(e.getKeyCode() == KeyEvent.VK_ESCAPE)
   System.exit(0);
  else if(e.getKeyCode() == KeyEvent.VK_SPACE){
   if(gamepause){
    t.suspend();
    gamepause = false;
   }
   else{
    t.resume();
    gamepause = true;
   }
  }
  else if(e.getKeyCode() == KeyEvent.VK_S){
   time--;
  }
  else if(e.getKeyCode() == KeyEvent.VK_D){
   time++;
  }
  
 }
 //generating random position of food
 int foodx = 20+(int)(Math.random()*56)*10;
 int foody = 20+(int)(Math.random()*52)*10;
 int score = 0;
 public void paintComponent(Graphics g){
  super.paintComponent(g);
  Graphics2D g2 = (Graphics2D)g;
  score = vector.size()-5;
  g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
  int red = 30+(int)(Math.random()*220);
  int green = 30+(int)(Math.random()*220);
  int blue = 30+(int)(Math.random()*220); 
  g2.setColor(Color.red);
  g2.drawRect(600, 0, 199, 550);
  g2.setColor(Color.orange);
  g2.setFont(new Font("bauhaus 93", 1, 30));
  g2.drawString("KHA SUCKS", 600, 80);
  g2.setFont(new Font("Arial", 1, 20));
  g2.drawString("Score: "+score,650, 230);
  g2.fillRect(0, 0, dim.width-200, dim.height);
  g2.setColor(Color.black);
  g2.fillRect(10, 10, dim.width-220, dim.height-65);
  g2.setColor(new Color(red, green, blue));
  g2.fillOval(foodx, foody, 10, 10);
  g2.setColor(Color.red);
                //if game over then it will be display on screen
  if(gameover){
   g2.setFont(new Font("Monotype Corsiva", 1, 120));
   g2.drawString("YOU SUCK", 60, 260);
   g2.setFont(new Font("typewriter",1,14));
   g2.drawString("This game is made by", 220, 330);
   g2.drawString("Torin Turner", 255, 370);
   g2.setColor(Color.yellow);
   g2.drawString("E-mail: Torinnturner@gmail.com", 170, 410);
   //t.stop();
  }
  else{   
   g2.fillOval(vector.get(0).centerx, vector.get(0).centery, vector.get(0).radius, vector.get(0).radius);
   for(int i=1; i < vector.size(); i++){
    g2.fillOval(vector.get(i).centerx, vector.get(i).centery, vector.get(i).radius, vector.get(i).radius);
    vector.get(i).pcenterx = vector.get(i).centerx;
    vector.get(i).pcentery = vector.get(i).centery;
    vector.get(i).centerx = vector.get(i-1).pcenterx;
    vector.get(i).centery = vector.get(i-1).pcentery;
   }
  }  
 }
        //important function which will see collision of snake with wall or itself
 void checkOutOfBounds(int x, int y){
  if(x < 10 || x > dim.width-220 || y < 10 || y > dim.height-61){
   gameover = true;
   try{Thread.sleep(1000);}catch(Exception e){}
   repaint();
  }
  else if(x==foodx && y==foody){
   snake_node snode = new snake_node();
   snode.centerx = vector.get(vector.size()-1).pcenterx;
   snode.centery = vector.get(vector.size()-1).pcentery;
   vector.add(snode);
   foodx = 20+(int)(Math.random()*56)*10;
   foody = 20+(int)(Math.random()*52)*10;   
  }
  for(int i=1; i < vector.size(); i++){
   if(x == vector.get(i).centerx && y == vector.get(i).centery){
    gameover = true;
    try{Thread.sleep(1000);}catch(Exception e){}
    repaint();
   }
  }
 }
 
 public void run(){
  
  while (true){
   switch(dir){
    case 'L':
     vector.get(0).centerx = (vector.get(0).centerx-inc);
     vector.get(0).pcenterx = vector.get(0).centerx;
     checkOutOfBounds(vector.get(0).centerx, vector.get(0).centery);
     repaint();
     break;
    case 'R':     
     vector.get(0).centerx = (vector.get(0).centerx+inc);
     vector.get(0).pcenterx = vector.get(0).centerx;
     checkOutOfBounds(vector.get(0).centerx, vector.get(0).centery);
     repaint();
     break;
    case 'U':     
     vector.get(0).centery = (vector.get(0).centery-inc);
     vector.get(0).pcentery = vector.get(0).centery;
     checkOutOfBounds(vector.get(0).centerx, vector.get(0).centery);
     repaint();
     break;
    case 'D':     
     vector.get(0).centery = (vector.get(0).centery+inc);
     vector.get(0).pcentery = vector.get(0).centery;
     checkOutOfBounds(vector.get(0).centerx, vector.get(0).centery);
     repaint();
     break;
   }
   try{
    Thread.sleep(time);
   }
   catch(Exception e){}
  }
 }
}
