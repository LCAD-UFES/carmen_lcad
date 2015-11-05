import processing.serial.*;

int[] accel_data = new int[3];
int[] adc_data = new int[16];
int desired_heading;
int desired_depth;
int desired_speed;
int[] marker_drop = new int[2];
int[] comp_accel = new int[3];
int[] comp_mag = new int[3];
int[] comp_heading = new int[3];
int[] comp_tilt = new int[3];
int[] battery = new int[4];
int[] pid = new int[10];
int opmode;
int gain_depth;
int gain_heading;
int output_depth;
int output_heading;
int integral_time;

Serial myPort;



void setup(){
  myPort = new Serial(this, Serial.list()[2], 57600);
//myPort = new Serial(this, "/dev/ttyUSB0", 115200); 
  println("setup done");
}

void draw(){

}

void keyReleased() {
 if(key == 'p'){
  readData();
  printOut();
 } 
 else if(key == 's'){
  setData(); 
 }
}

void setData(){
  
}

void readData(){
    myPort.clear();
    myPort.write(0x00);
    delay(50);
    accel_data[0] = myPort.read();
    accel_data[1] = myPort.read();
    accel_data[2] = myPort.read();
    adc_data[0] = myPort.read();
    adc_data[0] |= myPort.read()<<8;
    adc_data[1] = myPort.read();
    adc_data[1] |= myPort.read()<<8;
    adc_data[2] = myPort.read();
    adc_data[2] |= myPort.read()<<8;
    adc_data[3] = myPort.read();
    adc_data[3] |= myPort.read()<<8;
    adc_data[4] = myPort.read();
    adc_data[4] |= myPort.read()<<8;
    adc_data[5] = myPort.read();
    adc_data[5] |= myPort.read()<<8;
    adc_data[6] = myPort.read();
    adc_data[6] |= myPort.read()<<8;
    adc_data[7] = myPort.read();  
    adc_data[7] |= myPort.read()<<8;
    adc_data[8] = myPort.read();
    adc_data[8] |= myPort.read()<<8;
    adc_data[9] = myPort.read();
    adc_data[9] |= myPort.read()<<8;
    adc_data[10] = myPort.read();
    adc_data[10] |= myPort.read()<<8;
    adc_data[11] = myPort.read();
    adc_data[11] |= myPort.read()<<8;
    adc_data[12] = myPort.read();
    adc_data[12] |= myPort.read()<<8;
    adc_data[13] = myPort.read();
    adc_data[13] |= myPort.read()<<8;
    adc_data[14] = myPort.read();
    adc_data[14] |= myPort.read()<<8;
    adc_data[15] = myPort.read();  
    adc_data[15] |= myPort.read()<<8;
    desired_heading = myPort.read();
    desired_heading |= myPort.read()<<8;
    desired_depth = myPort.read();
    desired_speed = myPort.read();
    marker_drop[0] = myPort.read();
    marker_drop[1] = myPort.read();    
    comp_accel[0] = myPort.read();
    comp_accel[0] += 256*myPort.read();
    comp_accel[1] = myPort.read();
    comp_accel[1] += 256*myPort.read();
    comp_accel[2] = myPort.read();
    comp_accel[2] += 256*myPort.read();    
    comp_mag[0] = myPort.read();
    comp_mag[0] += 256*myPort.read();
    comp_mag[1] = myPort.read();
    comp_mag[1] += 256*myPort.read();
    comp_mag[2] = myPort.read();
    comp_mag[2] += 256*myPort.read();    
    comp_heading[0] = myPort.read();
    comp_heading[0] += 256*myPort.read();
    comp_heading[1] = myPort.read();
    comp_heading[1] += 256*myPort.read();
    comp_heading[2] = myPort.read();
    comp_heading[2] += 256*myPort.read();    
    comp_tilt[0] = myPort.read();
    comp_tilt[0] += 256*myPort.read();    
    comp_tilt[1] = myPort.read();
    comp_tilt[1] += 256*myPort.read();    
    comp_tilt[2] = myPort.read();
    comp_tilt[2] += 256*myPort.read();    
    battery[0] = myPort.read();
    battery[1] = myPort.read();
    battery[2] = myPort.read();
    battery[3] = myPort.read();    
    pid[0] = myPort.read();
    pid[1] = myPort.read();
    pid[2] = myPort.read();
    pid[3] = myPort.read();
    pid[4] = myPort.read();
    pid[5] = myPort.read();
    pid[6] = myPort.read();
    pid[7] = myPort.read();
    pid[8] = myPort.read();
    pid[9] = myPort.read();
}

void printOut(){
    println("======ACCELEROMETER========");
    println("accel[0]: "+accel_data[0]+"\t"+"accel[1]: "+accel_data[1]+"\t"+"accel[2]: "+accel_data[2]);
    println("=========ADC DATA==========");
    println("adc[0]: " + adc_data[0]+"\t"+"adc[1]: " + adc_data[1]);
    println("adc[2]: " + adc_data[2]+"\t"+"adc[3]: " + adc_data[3]);
    println("adc[4]: " + adc_data[4]+"\t"+"adc[5]: " + adc_data[5]);
    println("adc[6]: " + adc_data[6]+"\t"+"adc[7]: " + adc_data[7]); 
    println("adc[8]: " + adc_data[8]+"\t"+"adc[9]: " + adc_data[9]);
    println("adc[10]: " + adc_data[10]+"\t"+"adc[11]: " + adc_data[11]);
    println("adc[12]: " + adc_data[12]+"\t"+"adc[13]: " + adc_data[13]);
    println("adc[14]: " + adc_data[14]+"\t"+"adc[15]: " + adc_data[15]);
    println("==========COMPASS===========");
    println("heading : "+comp_heading[0]+"\t pitch: "+comp_heading[1]+"\t roll: "+comp_heading[2]);
    println("==========PID=========");
    println("Desired Heading: "+desired_heading);
    println("Heading KK: "+pid[0]+"\t KP: "+pid[1]+"\t KD: "+pid[2]+"\t KI: "+pid[3]);
    println("Heading Output: "+pid[4]);
    println("Desired Depth: "+desired_depth);
    println("Depth KK: "+pid[5]+"\t KP: "+pid[6]+"\t KD: "+pid[7]+"\t KI: "+pid[8]);
    println("Depth Output: "+pid[9]);
    
    println("\n");
}





