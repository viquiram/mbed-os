//#if 0

#include "mbed.h"
#include "stdio.h"
#include "string.h"
//#include "intro.c"
#include "clcd_def.h"
#include "icons.h"
#include "MPS2_RESOURCE_PACK.h"
//#include "./demo/demo1.h"

SPI spi_test(MOSI_SPI, MISO_SPI, SCLK_SPI);
MPS2CLCD clcd_test(CLCD_MOSI, CLCD_MISO, CLCD_SCLK);
DigitalOut slave_select(SSEL_SPI);
DigitalOut clcd_select(CLCD_SSEL);
MPS2AACI audio_test(AUD_SDA, AUD_SCL);
Serial pc(USBTX, USBRX); // tx, rx
MPS2Ethernet testing;

MPS2TSC touchscreen(TSC_SDA, TSC_SCL);

#define number_of_lines  2 //number of lines
#define shapesize  3  //pixels per side
#define angleremainder 3 //used to calculate the change in angle at edges of the screen

// Screen size
#define LCD_WIDTH           320         // Screen Width (in pixels)
#define LCD_HEIGHT          240         // Screen Height (in pixels)
#define TSC_I2C_ADDR          0x82

#define AACI_TIMEOUT       1000                                    // Timeout for reading FIFOs (10mS)


#define TX_PKT_SIZE 256
#define RX_PKT_SIZE 300

unsigned char txpkt[TX_PKT_SIZE];

unsigned char rxpkt[RX_PKT_SIZE];

unsigned char testpkt[] =
{
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xDE, 0xAD
};

	int count = 0;
	int lines_direction[number_of_lines][2];
	int value[number_of_lines][2];
	int textsize = 0;
  int tsc_readdata[2];
	int x_coord = 0;
	int y_coord = 0;
	int choice = 0;
	int done = 1;
	int z = rand();
 	int coords[number_of_lines][4];
  int hori_ctrl[number_of_lines];
  int vert_ctrl[number_of_lines];
	int colour[number_of_lines];
	int colourchoice[number_of_lines];
	short int randomcolour[number_of_lines];
	int worm_food[4];
	int food_seed[2];
	char worm_high_score[4][5];


void spiwrite(int mychar);
void direction_randomiser();
void touchscreen_read();
void main_menu ();
int exit_check();
int demo_exit_check(int button);
void worm_food_init();
int worm();
void setup();
//DEMO FUNCTIONS
// Demo 1 initialisation
void apDEMO1_init(void);
// Demo 1 test
void apDEMO1_run(void);
// Demo 2 Screen initialisation
void apDEMO2_init(void);
// Demo 2 test
void apDEMO2_run(void);
// Demo 3 Screen initialisation
void apDEMO3_init(void);
// Demo 3 test
void apDEMO3_run(void);
// Move slider vertical
void movesliderhori(unsigned int xpos, unsigned int ypos, unsigned int scol, unsigned int nxpos);
// Move slider vertical
void moveslidervert(unsigned int xpos, unsigned int ypos, unsigned int scol, unsigned int nypos);
unsigned int demo_mainmenu();
unsigned int ethernet_loopback_test();
unsigned int compare_buf(unsigned char *buf1, unsigned char *buf2, int len);


int main() {
	setup();
	while(1) {
		z = rand();
		srand(z);


		do{
			touchscreen_read();
			x_coord = tsc_readdata[0];
			y_coord = tsc_readdata[1];
			
			if ((11 < x_coord && x_coord <= 66) && (138 < y_coord && y_coord <= 192))
			{
				choice = 1; // line patterns
				done = 0;
				clcd_test.CLCD_bitmap(0, 0, 320, 240, (unsigned short *)lines_button);
				wait(1);
				clcd_test.CLCD_fillscreen(Black);
				clcd_test.CLCD_setbackcolor(Black);
				clcd_test.CLCD_settextcolor(White);
				char reset[] = "Reset";
				clcd_test.CLCD_displaystring(9,8,1,reset);

				
			}
			if ((86 < x_coord && x_coord <= 141) && (138 < y_coord && y_coord <= 192))
			{
				choice = 2; //demo
				done = 0;
				clcd_test.CLCD_bitmap(0, 0, 320, 240, (unsigned short *)demo_button);
				wait(1);
			}
			if ((180 < x_coord && x_coord <= 235) && (138 < y_coord && y_coord <= 192))
			{
				choice = 4; // worm
				done = 0;
				clcd_test.CLCD_bitmap(0, 0, 320, 240, (unsigned short *)worm_button);
				wait(1);

				
			}
			if ((255 < x_coord && x_coord <= 310) && (138 < y_coord && y_coord <= 192))
			{
				choice = 3; // other
				done = 0;
				clcd_test.CLCD_bitmap(0, 0, 320, 240, (unsigned short *)other_button);
				wait(1);
			}
			if ((38 < x_coord && x_coord <= 119) && (16 < y_coord && y_coord <= 97))
			{
				choice = 5; // other
				done = 0;
				
			}
			if ((199 < x_coord && x_coord <= 280) && (16 < y_coord && y_coord <= 97))
			{
				choice = 6; // other
				done = 0;
				
			}
		}while (done == 1);
		
		do{
			int exitcheck = 0;
			int demoexitcheck = 0;
			switch(choice)
			{
				case 0 :{ break;}
				case 1 : 
				{
					do
					{						
						for(int i = 0; i<=(number_of_lines-1); i++)
						{
							if(count == randomcolour[i]){
								int change = rand();
								colour[i]= (change%65536);
								if(colour[i] >=65536){colour[i]=0;}
							}
						}
						if(count == 65536)
						{
							count = 0;
						}
						if(count%16384 == 0)
						{
							direction_randomiser();
						}
					
						for(int i = 0; i<=(number_of_lines-1); i++)
						{
							if (coords[i][0] <= 0 || coords[i][1] >= 320) { hori_ctrl[i] = 1;}
							if (coords[i][0] >= 320 || coords[i][1] <= 0) {hori_ctrl[i] = 0;}
							
							if(hori_ctrl[i] == 0){coords[i][0]-=(lines_direction[i][0]+1); coords[i][1]+=(lines_direction[i][0]+1);}
							if(hori_ctrl[i] == 1){coords[i][0]+=(lines_direction[i][0]+1); coords[i][1]-=(lines_direction[i][0]+1);}
								
							if (coords[i][2] <= 10 || coords[i][3] >= 190) {vert_ctrl[i] = 1;}
							if (coords[i][2] >= 190 || coords[i][3] <= 10) {vert_ctrl[i] = 0;}
							if(vert_ctrl[i] == 0){coords[i][2]-=(lines_direction[i][1]+1); coords[i][3]+=(lines_direction[i][1]+1);}
							if(vert_ctrl[i] == 1){coords[i][2]+=(lines_direction[i][1]+1); coords[i][3]-=(lines_direction[i][1]+1);}
						}
		
						for(int i = 0; i<=(number_of_lines-1); i++)
						{
								/*clcd_test.CLCD_write(0x02, ( coords[i][0] >> 8));
								clcd_test.CLCD_write(0x03, ( coords[i][0] & 0xFF));
								clcd_test.CLCD_write(0x04, ((319 - coords[i][1]) >> 8));
								clcd_test.CLCD_write(0x05, ((319 - coords[i][1]) & 0xFF));
								clcd_test.CLCD_write(0x06, ( coords[i][2] >> 8));
								clcd_test.CLCD_write(0x07, ( coords[i][2] & 0xFF));
								clcd_test.CLCD_write(0x08, ((239 - coords[i][3]) >> 8));
								clcd_test.CLCD_write(0x09, ((239 - coords[i][3]) & 0xFF));
								clcd_test.CLCD_write_cmd(0x22);
								
								clcd_test.CLCD_master_write_start();
								for(int loop = 0; loop < (shapesize*shapesize); loop++){
									clcd_test.CLCD_write_fw(thecolours[colour[i]]);
								}
								clcd_test.CLCD_master_write_stop();*/
								clcd_test.CLCD_boxsize(coords[i][0],(320 - (coords[i][1])),coords[i][2],(200 - (coords[i][3])),thecolours[colour[i]]);
						}
						count++;
					} while ((exitcheck = exit_check())!=1);
					if(exitcheck == 1){break;}
				}
				case 2 :	
				{
					unsigned int x, y, button;
					unsigned short col;
					// Run the demos
					do
					{
						// Get main menu selection
						button = demo_mainmenu();
			
						// Mirror button press
						clcd_test.CLCD_setwindowsize(0, 319, 64, 239);
						clcd_test.CLCD_master_write_start();
						for(y = 64; y < 240; y++)
						{
									for(x = 0; x < 320; x++)
									{
										col = flyerData[x + (y * 320)];
										if ((x > 244) && (y > (64 + (button * 58))) && (y < (122 + (button * 58))) && (col == 0xFFFF))
										{
											clcd_test.CLCD_write_fw(0x07E0);
										}else
										{
										clcd_test.CLCD_write_fw(col);
										}
									}
						}
						clcd_test.CLCD_master_write_stop();
						wait_ms(1000);
			
					// Run demos
					if (button == 0)
						apDEMO1_run();
					else if (button == 1)
						apDEMO2_run();
					else if (button == 2)
						apDEMO3_run();
			
					} while ((demoexitcheck = demo_exit_check(button)) != 1);
					if(demoexitcheck == 1)
					{
						button = 0;
						wait(2);
						break;
					}
				}
				case 3 :
				{ 
					pc.printf("inside case 3 doing ethernet testing\n");
					char macaddr[12];
					for( int m = 0; m<12;m+=2)
					{
						macaddr[m] = (char) m/2;
						macaddr[m+1] = (char) m/2;
					}
					int tries, i;//, failtest;
//					failtest = FALSE;

					clcd_test.CLCD_fillscreen(Black);
					
					;
					if(testing.Ethernet_init()) 
					{
						pc.printf("SMSC9220 initialisation failed.\n\n");
					}
					
					if(testing.mac_address(macaddr))
					{
		        printf("Invalid MAC Address. Selftest was unable to change MAC address.\n\n");
					}

					
					if(testing.Ethernet_check_ready()) 
					{
						pc.printf("Error: Ready bit not set.\n");
					} else {
						pc.printf("Ready bit is set.\n");
					}
					
					NVIC_ClearPendingIRQ(ETHERNET_IRQn);

					printf("Data loop back test...\n");
					
					for(tries = 0; tries < 1000; tries++)
					{
						//pc.printf("data is ");
						for(i=0; i < 16; i++)
						{
							txpkt[i] = testpkt[i];
							//pc.printf("%d, ",testpkt[i]);
							//pc.printf("%d | ",txpkt[i]);
							
						}
						//pc.printf("\n");
						//pc.printf("data is ");
						for( ; i < TX_PKT_SIZE; i++)
						{
							txpkt[i++] = tries & 0xff;
							txpkt[i] = ((i-16)>>1) & 0xff;
							//pc.printf("%d, ",testpkt[i]);
						}
						//pc.printf("\n");
						if(ethernet_loopback_test()) 
						{
							//pc.printf("Error: Loopback test failed on transfer %d\n",tries+1);
							//pc.printf("Transmitted data is %X \n",txpkt);
							//pc.printf("Received data is %X \n",rxpkt);
//							failtest = TRUE;
							break;
						} else {
							//pc.printf("Transfer %d successful.\n",tries+1);
						}
						
						memset(rxpkt, 0, RX_PKT_SIZE);
						//pc.printf("Received data is %X \n",rxpkt);
					}
					/*smsc9220_set_soft_int();	   // Generate a soft irq.

					if (ap_check_peripheral_interrupt("Ethernet", ETHERNET_IRQn, 1))
					{
						failtest = TRUE;
					}
			
					smsc9220_clear_soft_int();     // Clear soft irq.*/
					pc.printf("Ethernet Loopback test successful. Loopback carried out %d.\n",tries);
			
					NVIC_ClearPendingIRQ(ETHERNET_IRQn);

					//Under construction screen
					// Fill display with test pattern bitmap (320*240*16bit RGB 5:6:5)
					clcd_test.CLCD_bitmap(0, 0, 320, 240, (unsigned short *)otherscreen);
					do
					{	//clcd_test.CLCD_fillscreen(Black);
						clcd_test.CLCD_setbackcolor(0xE73C);
						clcd_test.CLCD_settextcolor(Black);
						char goback[] = "Menu";
						clcd_test.CLCD_displaystring(9,8,1,goback);

					} while ((exitcheck = exit_check())!=1);
					if(exitcheck == 1){break;}
				}
				case 4 : 
				{
					int gameover = 0;
					clcd_test.CLCD_fillscreen(Black);
					clcd_test.CLCD_box(40,40,240,160,LightGrey);
					char movement_left[] = "LEFT";
					char movement_right[] = "RIGHT";
					char movement_up[] = "UP";
					char movement_down[] = "DOWN";

					clcd_test.CLCD_setbackcolor(Black);
					clcd_test.CLCD_settextcolor(White);
					clcd_test.CLCD_displaystring(0,9,1,movement_up);
					clcd_test.CLCD_displaystring(9,8,1,movement_down);
					for(int a = 0; a <=3; a++){
						clcd_test.CLCD_displaychar(a+3,1,1,movement_left[a]);
					}
					for(int a = 0; a <=4; a++){
					clcd_test.CLCD_displaychar(a+3,19,1,movement_right[a]);
					}
					//playing worm
					do
					{
						gameover = worm();
					}while(!gameover);
					if(gameover == 1){
						clcd_test.CLCD_fillscreen(Black);
						//high score calculation for worm
						int temp1[5] ,temp2[5];
						for(int y = 0; y<=4; y++)
						{
							temp1[y] = 0;
							temp2[y] = 0;
						}
						for(int u = 2; u>=0; u --)
						{
							for(int y = 4; y>=0; y--)
							{
								temp1[y] = (unsigned int)worm_high_score[u][y]-48;
								temp2[y] = (unsigned int)worm_high_score[u+1][y]-48;
							}
							int bigger = 0;
							int w = 0;
							do{
								switch (w) {
									case 0 : {
										if (temp1[w] == temp2[w]){}
										else if (temp1[w] < temp2[w]){bigger +=5;}
										else if(temp1[w] > temp2[w]){bigger -= 5;}
										break;
									}
									case 1 : {
										if (temp1[w] == temp2[w]){}
										else if (temp1[w] < temp2[w]){bigger +=4;}
										else if(temp1[w] > temp2[w]){bigger -= 4;}
										break;
									}
									case 2 : {
										if (temp1[w] == temp2[w]){}
										else if (temp1[w] < temp2[w]){bigger +=3;}
										else if(temp1[w] > temp2[w]){bigger -= 3;}
										break;
									}
									case 3 : {
										if (temp1[w] == temp2[w]){}
										else if (temp1[w] < temp2[w]){bigger +=2;}
										else if(temp1[w] > temp2[w]){bigger -= 2;}
										break;
									}
									case 4 : {
										if (temp1[w] == temp2[w]){}
										else if (temp1[w] < temp2[w]){bigger +=1;}
										else if(temp1[w] > temp2[w]){bigger -= 1;}
										break;
									}
								}
								w++;
							}while(w < 5);
							if(bigger > 0)
							{
								worm_high_score[u][0] = (0x30 + temp2[0]);
								worm_high_score[u][1] = (0x30 + temp2[1]);
								worm_high_score[u][2] = (0x30 + temp2[2]);
								worm_high_score[u][3] = (0x30 + temp2[3]);
								worm_high_score[u][4] = (0x30 + temp2[4]);
								worm_high_score[u+1][0] = (0x30 + temp1[0]);
								worm_high_score[u+1][1] = (0x30 + temp1[1]);
								worm_high_score[u+1][2] = (0x30 + temp1[2]);
								worm_high_score[u+1][3] = (0x30 + temp1[3]);
								worm_high_score[u+1][4] = (0x30 + temp1[4]);
							}
						}
					
					}
						//end screen with top scores for worm
						char topscoretext[] = "TOP SCORES";
						char gameovertext[] = "GAME OVER";
						char numberone[] = "#1 ";
						char numbertwo[] = "#2 ";
						char numberthree[] = "#3 ";
					
						clcd_test.CLCD_box(5,5,310,200,Red);
						clcd_test.CLCD_setbackcolor(Red);
						clcd_test.CLCD_settextcolor(Black);
						clcd_test.CLCD_displaystring(2,5,1,topscoretext);
						clcd_test.CLCD_displaystring(3,5,1,numberone);
						clcd_test.CLCD_displaychar(3,8,1, worm_high_score[0][0]);
						clcd_test.CLCD_displaychar(3,9,1, worm_high_score[0][1]);
						clcd_test.CLCD_displaychar(3,10,1,worm_high_score[0][2]);
						clcd_test.CLCD_displaychar(3,11,1,worm_high_score[0][3]);
						clcd_test.CLCD_displaychar(3,12,1,worm_high_score[0][4]);
						clcd_test.CLCD_displaystring(4,5,1,numbertwo);
						clcd_test.CLCD_displaychar(4,8,1, worm_high_score[1][0]);
						clcd_test.CLCD_displaychar(4,9,1, worm_high_score[1][1]);
						clcd_test.CLCD_displaychar(4,10,1,worm_high_score[1][2]);
						clcd_test.CLCD_displaychar(4,11,1,worm_high_score[1][3]);
						clcd_test.CLCD_displaychar(4,12,1,worm_high_score[1][4]);
						clcd_test.CLCD_displaystring(5,5,1,numberthree);
						clcd_test.CLCD_displaychar(5,8,1, worm_high_score[2][0]);
						clcd_test.CLCD_displaychar(5,9,1, worm_high_score[2][1]);
						clcd_test.CLCD_displaychar(5,10,1,worm_high_score[2][2]);
						clcd_test.CLCD_displaychar(5,11,1,worm_high_score[2][3]);
						clcd_test.CLCD_displaychar(5,12,1,worm_high_score[2][4]);
						//clcd_test.CLCD_displaystring(4,8,1,worm_high_score[1]);
						//clcd_test.CLCD_displaystring(5,8,1,worm_high_score[2]);
						clcd_test.CLCD_setbackcolor(Black);
						clcd_test.CLCD_settextcolor(White);
						char mainmenubut[] = "Main Menu";
						clcd_test.CLCD_displaystring(9,6,1,mainmenubut);
						clcd_test.CLCD_setbackcolor(Red);
						clcd_test.CLCD_settextcolor(Black);
					do{
						clcd_test.CLCD_displaystring(7,6,1,gameovertext);
						wait(0.1);
					}while((exitcheck = exit_check())!=1);
					if(exitcheck == 1){break;}
				}
				case 5 : 
				{
					clcd_test.CLCD_fillscreen(Black);
					// Fill display with test pattern bitmap (320*240*16bit RGB 5:6:5)
					clcd_test.CLCD_bitmap(0, 0, 320, 240, (unsigned short *)armtag);
					do
					{	//clcd_test.CLCD_fillscreen(Black);
						clcd_test.CLCD_setbackcolor(0xE73C);
						clcd_test.CLCD_settextcolor(Black);
						char goback[] = "Menu";
						clcd_test.CLCD_displaystring(9,8,1,goback);

					} while ((exitcheck = exit_check())!=1);
					if(exitcheck == 1){break;}
				}
				case 6 : 
				{
						clcd_test.CLCD_fillscreen(Black);
						// Fill display with test pattern bitmap (320*240*16bit RGB 5:6:5)
					clcd_test.CLCD_bitmap(0, 0, 320, 240, (unsigned short *)mbedtag);
					do
					{	//clcd_test.CLCD_fillscreen(Black);
						clcd_test.CLCD_setbackcolor(White);
						clcd_test.CLCD_settextcolor(Black);
						char goback[] = "Menu";
						clcd_test.CLCD_displaystring(9,8,1,goback);

					} while ((exitcheck = exit_check())!=1);
					if(exitcheck == 1){break;}
				}

			}
		}while (done != 1);
	}
}
void spiwrite(int mychar)
{
			slave_select = 0;
			spi_test.write(mychar);
			slave_select = 1;
			wait_ms(10);
}

void direction_randomiser(){
			for(int i = 0; i<=(number_of_lines-1); i=i+4){
			for(int k = 0; k<=1; k++){
				value[i][k] += rand();
				value[i][k] += rand();
				if(value[i][k]<0){value[i][k]= value[i][k]-(2*value[i][k]);}
				
				lines_direction[i][k] = value[i][k]%angleremainder;
			}
		}
	
		for(int i = 0; i<=(number_of_lines-1); i=i+4){
		for(int k = 0; k<=1; k++){
			lines_direction[i+1][k] = lines_direction[i][k];
			lines_direction[i+2][k] = lines_direction[i][k];
			lines_direction[i+3][k] = lines_direction[i][k];
		}
	}
}

void touchscreen_read()
{
	  unsigned int loop, din, isr;//, done;
		unsigned int X, Y, XPXL, YPXL, XPXLN, YPXLN, pendown;

		pendown  = FALSE;

    	// Read interrupt status
    	isr = touchscreen.TSC_READ(0x0B, TSC_I2C_ADDR, 1);
			
			int thres = touchscreen.TSC_READ(0x4A,TSC_I2C_ADDR,1);

    	// Clear pen down if state changes
    	if (isr & 0x01)
    		pendown = FALSE;

    	// Test for FIFO_TH interrupt
    	if (isr & 0x02)
    	{
			// Empty the FIFO
			loop = touchscreen.TSC_READ(0x4C, TSC_I2C_ADDR, 1);
			while (loop > 1)
			{
				din = touchscreen.TSC_READ(0xD7, TSC_I2C_ADDR, 4);
				loop--;
			}

			// Clear the interrupt (must be immediately after FIFO empty)
			touchscreen.TSC_WRITE(0x0B, isr, TSC_I2C_ADDR);

			// Read coordinates
			din = touchscreen.TSC_READ(0xD7, TSC_I2C_ADDR, 4);
			X = (din >> 20) & 0x00000FFF;
			Y = (din >>  8) & 0x00000FFF;

			// Calculate the pixel position (X/Y are swapped)
			XPXL = LCD_WIDTH - ((Y * 10) / (TSC_MAXVAL / LCD_WIDTH)) + TSC_XOFF;
			YPXL = ((X * 10) / (TSC_MAXVAL / LCD_HEIGHT)) - TSC_YOFF;
			XPXL = (XPXL & 0x80000000) ? 0 : XPXL;
			YPXL = (YPXL & 0x80000000) ? 0 : YPXL;
			XPXL = (XPXL >= LCD_WIDTH ) ? LCD_WIDTH  - 1 : XPXL;
			YPXL = (YPXL >= LCD_HEIGHT) ? LCD_HEIGHT - 1 : YPXL;

			// Move to new position if pen lifted
			if (!pendown)
			{
				XPXLN = XPXL;
				YPXLN = YPXL;
			}
			//clcd_test.CLCD_settextcolor(White);
			// Display line to new pixel position
			do
			{
				// Update X and Y
				XPXLN = (XPXL > XPXLN) ? XPXLN + 1 : XPXLN;
				XPXLN = (XPXL < XPXLN) ? XPXLN - 1 : XPXLN;
				YPXLN = (YPXL > YPXLN) ? YPXLN + 1 : YPXLN;
				YPXLN = (YPXL < YPXLN) ? YPXLN - 1 : YPXLN;

				// Display pixel
			} while ((XPXLN != XPXL) || (YPXLN != YPXL));

			// Clear display
			if ((XPXL < 80) && (YPXL > 200))
			{
			}

			// Done
			if ((XPXL > 238) && (YPXL > 200))

			// Save pen state
    		pendown = TRUE;
    	}
    	else

    	// Clear status register
    	if (isr)
    		 touchscreen.TSC_WRITE(0x0B, isr, TSC_I2C_ADDR);


    //} while (!done);
		if(XPXL > 320){ tsc_readdata[0] = 320; }
		else { tsc_readdata[0] = XPXL;}
		
		if(YPXL > 240){ tsc_readdata[1] = 240; }
		else { tsc_readdata[1] = YPXL; }
		
	}

// Move slider vertical
void movesliderhori(unsigned int xpos, unsigned int ypos, unsigned int scol, unsigned int nxpos)
{
	unsigned int loop, col;

	// Clear the old slider
    clcd_test.CLCD_boxsize(xpos - 10, xpos + 5, ypos, ypos + 23, 0xFFFF);

	// Draw the new slider
	clcd_test.CLCD_setwindowsize(nxpos - 10, nxpos + 5, ypos, ypos + 23);
	clcd_test.CLCD_master_write_start();
	for(loop = 0; loop < (16*23); loop++)
	{
		col = (sliderData[loop] & 0xFFFF);
		if(col == 0x0000)
			clcd_test.CLCD_write_fw(scol);
		else
			clcd_test.CLCD_write_fw(col);
	}
	clcd_test.CLCD_master_write_stop();
}

// Move slider vertical
void moveslidervert(unsigned int xpos, unsigned int ypos, unsigned int scol, unsigned int nypos)
{
	unsigned int loop, col;

	// Clear the old slider
    clcd_test.CLCD_boxsize(xpos, xpos + 23, ypos, ypos + 15, 0xFFFF);

	// Draw the new slider
	clcd_test.CLCD_setwindowsize(xpos, xpos + 23, nypos, nypos + 15);
	clcd_test.CLCD_master_write_start();
	for(loop = 0; loop < (23*16); loop++)
	{
		col = (slidervData[loop] & 0xFFFF);
		if(col == 0x0000)
			clcd_test.CLCD_write_fw(scol);
		else
			clcd_test.CLCD_write_fw(col);
	}
	clcd_test.CLCD_master_write_stop();
}

unsigned int demo_mainmenu()
{
	unsigned int state, pend, penx, peny, button, timer;

	clcd_test.CLCD_fillscreen(Black);
 
	// Fill display with test pattern bitmap (320*240*16bit RGB 5:6:5)
	clcd_test.CLCD_bitmap(0, 0, 320, 240, (unsigned short *)flyerData);

	// Wait for button press
	state  = 0;
	button = 0;
	timer  = 0;
    do
    {
		// Draw PRESS screen image box (8*74)
    	if (!timer)
    	{
            if(!state)
                clcd_test.CLCD_bitmap(231, 110, 8, 74, (unsigned short *)pressData);
            else
                clcd_test.CLCD_box (231, 110, 8, 74, 0x0BF3);
            
			state = state ^ 0x01;
    	}

		// Read touchscreen
		pend = touchscreen.TSC_READXY(&penx, &peny);

		// Check for button press
		if (pend)
		{
			if((penx > 250) && (penx < 311))
			{
				if ((peny > 69) && (peny < 117))
					button = 1;
				if ((peny > 127) && (peny < 175))
					button = 2;
				if ((peny > 185) && (peny < 233))
					button = 3;
			}
			if((penx > 5) && (penx < 100))
			{
				button = 8;
			}
		}

		// Update PRESS every 500mS
		timer = (timer > 50) ? 0 : timer + 1;
		wait_ms(10);
    } while (!button);

    return (button - 1);
}

	
	
void main_menu ()
{
	clcd_test.CLCD_fillscreen(Black);
	// Fill display with test pattern bitmap (320*240*16bit RGB 5:6:5)
  clcd_test.CLCD_bitmap(0, 0, 320, 240, (unsigned short *)mainmenu);
}

int exit_check()
{
	touchscreen_read();
	x_coord = tsc_readdata[0];
	y_coord = tsc_readdata[1];
  
	if ( y_coord >= 200)
	{
		main_menu();
		done = 1;
		wait(2);
		choice = 0;
		x_coord = 0;
		y_coord = 0;
		tsc_readdata[0] = 0;		
		tsc_readdata[1] = 0;		
		return 1;
	} else {
		return 0;
	}
}

int demo_exit_check(int button)
{
	if ( button == 7)
	{
		main_menu();
		done = 1;
		wait(2);
		choice = 0;
		x_coord = 0;
		y_coord = 0;
		tsc_readdata[0] = 0;		
		tsc_readdata[1] = 0;		
		return 1;
	} else {
		return 0;
	}
}

void worm_food_init()
{
		food_seed[0] = rand();
	food_seed[1] = rand();
	
	while (food_seed[0] > 220) {
		if(food_seed[0] >= 220) {
			food_seed[0] = food_seed[0]/10;
		}
	}
	while (food_seed[1] > 150) {
		if(food_seed[1] >= 150) {
			food_seed[1] = food_seed[1]/10;
		}
	}
	worm_food[0] = 40 + food_seed[0];
	worm_food[1] = 275 - food_seed[0];
	worm_food[2] = 40 + food_seed[1];
	worm_food[3] = 195 - food_seed[1];
}
int worm()
{
	int worm_head[4]; // [0] from left edge, [1] from right edge [2] from top [3] from bottom
	int worm_tail[4]; // [0] horizontal postion, [1] vertical postion
	int worm_body[1000][4]; // store of the last 100 head coords
	int worm_direction = 1; //  0 = left, 1 = right, 2 = up, 3 = down
	int game_over = 0;
	int length = 20;
	int move_tail = 0;
	double delay = 0.01;
	int food_eaten = 0;
	int food_check[4];
	unsigned long int score = 0;
	
	//initialisation of worm head at start position
	for(int y = 0; y <=3; y++)
	{
		if(y == 0){worm_head[y] = 40; worm_body[0][y] = worm_head[y]; }
		else if(y == 1){worm_head[y] = 275; worm_body[0][y] = worm_head[y];}
		else if(y == 2){worm_head[y] = 80; worm_body[0][y] = worm_head[y];}
		else if(y == 3){worm_head[y] = 155; worm_body[0][y] = worm_head[y];}
	}

	//initialisation of worm body 
	for(int h = 1; h <= 999; h++)
	{
		for(int j=0; j<=3; j++)
		{
			worm_body[h][j] = worm_body[h-1][j];
		}
	}
		worm_food_init();
		//first food position			
		/*clcd_test.CLCD_write(0x02, ( worm_food[0] >> 8));
		clcd_test.CLCD_write(0x03, ( worm_food[0] & 0xFF));
		clcd_test.CLCD_write(0x04, ((319 - worm_food[1]) >> 8));
		clcd_test.CLCD_write(0x05, ((319 - worm_food[1]) & 0xFF));
		clcd_test.CLCD_write(0x06, ( worm_food[2] >> 8));
		clcd_test.CLCD_write(0x07, ( worm_food[2] & 0xFF));
		clcd_test.CLCD_write(0x08, ((239 - worm_food[3]) >> 8));
		clcd_test.CLCD_write(0x09, ((239 - worm_food[3]) & 0xFF));
		clcd_test.CLCD_write_cmd(0x22);
		
		clcd_test.CLCD_master_write_start();
		for(int loop = 0; loop < (5*5); loop++){
			clcd_test.CLCD_write_fw(Red);
		}
		clcd_test.CLCD_master_write_stop();*/
		clcd_test.CLCD_boxsize(worm_food[0],(320 - worm_food[1]),worm_food[2],(240 - worm_food[3]),Red);

	
		do
		{
			//check if food has been eaten
			for(int j=0; j<=3; j++)
			{
				if(	worm_head[j] == (worm_food[j] - 4)
				 || worm_head[j] == (worm_food[j] - 3)
				 || worm_head[j] == (worm_food[j] - 2)
				 || worm_head[j] == (worm_food[j] - 1)
				 || worm_head[j] == (worm_food[j]		 )
				 || worm_head[j] == (worm_food[j] + 1)
				 || worm_head[j] == (worm_food[j] + 2)
				 || worm_head[j] == (worm_food[j] + 3)
				 || worm_head[j] == (worm_food[j] + 4))
				{food_check[j] = 1;}else{ food_check[j] = 0;}
			}				
			if(food_check[0] && food_check[1] && food_check[2] && food_check[3])
			{ food_eaten = 1;}else{food_eaten = 0;}
			
			//if food eaten, remove food in old position
			if(food_eaten == 1)
			{
				/*clcd_test.CLCD_write(0x02, ( worm_food[0] >> 8));
				clcd_test.CLCD_write(0x03, ( worm_food[0] & 0xFF));
				clcd_test.CLCD_write(0x04, ((319 - worm_food[1]) >> 8));
				clcd_test.CLCD_write(0x05, ((319 - worm_food[1]) & 0xFF));
				clcd_test.CLCD_write(0x06, ( worm_food[2] >> 8));
				clcd_test.CLCD_write(0x07, ( worm_food[2] & 0xFF));
				clcd_test.CLCD_write(0x08, ((239 - worm_food[3]) >> 8));
				clcd_test.CLCD_write(0x09, ((239 - worm_food[3]) & 0xFF));
				clcd_test.CLCD_write_cmd(0x22);
				
				clcd_test.CLCD_master_write_start();
				for(int loop = 0; loop < (5*5); loop++){
					clcd_test.CLCD_write_fw(LightGrey);
				}
				clcd_test.CLCD_master_write_stop();*/
				clcd_test.CLCD_boxsize(worm_food[0],(320 - worm_food[1]),worm_food[2],(240 - worm_food[3]),LightGrey);
				
				
				
				//generate new food position away from worm
				int new_food_bad[4];
				int new_food_good[4];
				int bad_new_food = 0;
				int good_new_food = 1;
				for(int j=0; j<=3; j++)
				{
					new_food_bad[j] = 0;
					new_food_good[j] = 1;
				}
				do
				{
					worm_food_init();
						for(int h = 1; h <length; h++)
						{
							for(int j=0; j<=3; j++)
							{
								if(	worm_food[j] == (worm_body[h][j] - 4 )
									|| worm_food[j] == (worm_head[j] - 4	 )
									|| worm_food[j] == (worm_tail[j] - 4	 )
									|| worm_food[j] == (worm_body[h][j] - 3)
									|| worm_food[j] == (worm_head[j] - 3	 )
									|| worm_food[j] == (worm_tail[j] - 3	 )
									|| worm_food[j] == (worm_body[h][j] - 2)
									|| worm_food[j] == (worm_head[j] - 2	 )
									|| worm_food[j] == (worm_tail[j] - 2	 )
									|| worm_food[j] == (worm_body[h][j] - 1)
									|| worm_food[j] == (worm_head[j] - 1	 )
									|| worm_food[j] == (worm_tail[j] - 1	 )
									|| worm_food[j] == (worm_body[h][j]		 )
									|| worm_food[j] == (worm_head[j]			 )
									|| worm_food[j] == (worm_tail[j]			 )
									|| worm_food[j] == (worm_body[h][j] + 1)
									|| worm_food[j] == (worm_head[j] + 1	 )
									|| worm_food[j] == (worm_tail[j] + 1	 )
									|| worm_food[j] == (worm_body[h][j] + 2)
									|| worm_food[j] == (worm_head[j] + 2	 )
									|| worm_food[j] == (worm_tail[j] + 2	 )
									|| worm_food[j] == (worm_body[h][j] + 3)
									|| worm_food[j] == (worm_head[j] + 3	 )
									|| worm_food[j] == (worm_tail[j] + 3	 )
									|| worm_food[j] == (worm_body[h][j] + 4)
									|| worm_food[j] == (worm_head[j] + 4	 )
									|| worm_food[j] == (worm_tail[j] + 4	 ))
								{
									new_food_bad[j] = 1;
									new_food_good[j] = 0;
									break;
								}
								else
								{
									new_food_good[j] = 1;
									new_food_bad[j] = 0;
								}
							}
							if((new_food_bad[0] && new_food_bad[2]) ||
									(new_food_bad[0] && new_food_bad[3]) ||
									(new_food_bad[1] && new_food_bad[2]) ||
									(new_food_bad[1] && new_food_bad[3]))
							{
								bad_new_food = 1;
								good_new_food = 0;
							}
							else
							{
							 if((new_food_good[0] && new_food_good[1] && new_food_good[2]) ||
									(new_food_good[1] && new_food_good[2] && new_food_good[3]) ||
									(new_food_good[0] && new_food_good[2] && new_food_good[3]) ||
									(new_food_good[0] && new_food_good[1] && new_food_good[3]))
								bad_new_food = 0;
								good_new_food = 1;
							}

							if(bad_new_food == 1)
							{
								break;
							}
						}
				}while(good_new_food != 1);
				//increase score by 1 and store
				if (score <= 99999)
				{
					score++;
				} else {
					score = 0;
				}
				worm_high_score[3][0] = (0x30 + ((score%100000)/10000));
				worm_high_score[3][1] = (0x30 + ((score%10000)/1000));
				worm_high_score[3][2] = (0x30 + ((score%1000)/100));
				worm_high_score[3][3] = (0x30 + ((score%100)/10));
				worm_high_score[3][4] = (0x30 + (score%10));
				clcd_test.CLCD_displaychar(0,15,1, worm_high_score[3][0]);
				clcd_test.CLCD_displaychar(0,16,1, worm_high_score[3][1]);
				clcd_test.CLCD_displaychar(0,17,1,worm_high_score[3][2]);
				clcd_test.CLCD_displaychar(0,18,1,worm_high_score[3][3]);
				clcd_test.CLCD_displaychar(0,19,1,worm_high_score[3][4]);
				//every five points modify speed and length of worm
				if ((score%5 == 0) && (score > 0))
				{
					length = length + 20 ;
					delay -= (delay / 5);
				}
				//draw food in new position			
				/*clcd_test.CLCD_write(0x02, ( worm_food[0] >> 8));
				clcd_test.CLCD_write(0x03, ( worm_food[0] & 0xFF));
				clcd_test.CLCD_write(0x04, ((319 - worm_food[1]) >> 8));
				clcd_test.CLCD_write(0x05, ((319 - worm_food[1]) & 0xFF));
				clcd_test.CLCD_write(0x06, ( worm_food[2] >> 8));
				clcd_test.CLCD_write(0x07, ( worm_food[2] & 0xFF));
				clcd_test.CLCD_write(0x08, ((239 - worm_food[3]) >> 8));
				clcd_test.CLCD_write(0x09, ((239 - worm_food[3]) & 0xFF));
				clcd_test.CLCD_write_cmd(0x22);
				
				clcd_test.CLCD_master_write_start();
				for(int loop = 0; loop < (5*5); loop++){
					clcd_test.CLCD_write_fw(Red);
				}
				clcd_test.CLCD_master_write_stop();*/
				clcd_test.CLCD_boxsize(worm_food[0],(320 - worm_food[1]),worm_food[2],(240 - worm_food[3]),Red);
      
			}
			
			for(int h = length; h >= 1; h--)
			{
				for(int j=0; j<=3; j++)
				{
					worm_body[h][j] = worm_body[h-1][j];
				}
			}
			for(int j=0; j<=3; j++)
			{
				worm_body[0][j] = worm_head[j];
				worm_tail[j] = worm_body[length][j];
			}
			
			touchscreen_read();
			x_coord = tsc_readdata[0];
			y_coord = tsc_readdata[1];
			if(y_coord < 120 && y_coord > 5)
			{
				if (x_coord>y_coord && x_coord < (320-y_coord))
				{
					if(worm_direction != 2){worm_direction = 2;}
				}
				else if ( x_coord<y_coord)
				{
					if(worm_direction != 0){worm_direction = 0;}
				}
				else if (x_coord>(320-y_coord))
				{
					if(worm_direction != 1){worm_direction = 1;}
				}
			}
			else if (y_coord >=120 && y_coord < 235)
			{
				if (x_coord > (240 - y_coord) && x_coord < (320-(240 - y_coord)))
				{
					if(worm_direction != 3){worm_direction = 3;}
				} 
				else if ( x_coord <= (240 - y_coord))
				{
					if(worm_direction != 0){worm_direction = 0;}
				}
				else if (x_coord >=(320-(240 - y_coord)))
				{
					if(worm_direction != 1){worm_direction = 1;}
				}
			}
			
			//check to see if worm hit the edge of the screen
			if (worm_head[0] < 40 || worm_head[1] >= 280) {game_over = 1; break;}
			if (worm_head[0] >= 280 || worm_head[1] < 40) {game_over = 1; break;}
			if (worm_head[2] < 40 || worm_head[3] >= 200) {game_over = 1; break;}
			if (worm_head[2] >= 200 || worm_head[3] < 40) {game_over = 1; break;}
			// modify worm head position dependant on direction
			if(worm_direction == 0){worm_head[0]-= 1; worm_head[1]+= 1;}
			if(worm_direction == 1){worm_head[0]+= 1; worm_head[1]-= 1;}
			if(worm_direction == 2){worm_head[2]-= 1; worm_head[3]+= 1;}
			if(worm_direction == 3){worm_head[2]+= 1; worm_head[3]-= 1;}
			
			//head position change			
			/*clcd_test.CLCD_write(0x02, ( worm_head[0] >> 8));
			clcd_test.CLCD_write(0x03, ( worm_head[0] & 0xFF));
			clcd_test.CLCD_write(0x04, ((319 - worm_head[1]) >> 8));
			clcd_test.CLCD_write(0x05, ((319 - worm_head[1]) & 0xFF));
			clcd_test.CLCD_write(0x06, ( worm_head[2] >> 8));
			clcd_test.CLCD_write(0x07, ( worm_head[2] & 0xFF));
			clcd_test.CLCD_write(0x08, ((239 - worm_head[3]) >> 8));
			clcd_test.CLCD_write(0x09, ((239 - worm_head[3]) & 0xFF));
			clcd_test.CLCD_write_cmd(0x22);
			
			clcd_test.CLCD_master_write_start();
			for(int loop = 0; loop < 25; loop++){
				clcd_test.CLCD_write_fw(Black);
			}
			clcd_test.CLCD_master_write_stop();*/
			clcd_test.CLCD_boxsize(worm_head[0],(320 - worm_head[1]),worm_head[2],(240 -worm_head[3]),Black);
			wait_ms(10);
			switch(worm_direction)
			{
				case 0 : {if((worm_tail[0] <= 275) && (worm_tail[1] >= 45 )) { move_tail = 1;} break;}
				case 1 : {if((worm_tail[0] >= 40 ) && (worm_tail[1] <= 275)){ move_tail = 1;} break;}
				case 2 : {if((worm_tail[2] <= 195) && (worm_tail[3] >= 45 )) { move_tail = 1;} break;}
				case 3 : {if((worm_tail[2] >= 40 ) && (worm_tail[3] <= 195)){ move_tail = 1;} break;}
			}
			if(move_tail == 1)
			{
				//tail position change			
				/*clcd_test.CLCD_write(0x02, ( worm_tail[0] >> 8));
				clcd_test.CLCD_write(0x03, ( worm_tail[0] & 0xFF));
				clcd_test.CLCD_write(0x04, ((319 - worm_tail[1]) >> 8));
				clcd_test.CLCD_write(0x05, ((319 - worm_tail[1]) & 0xFF));
				clcd_test.CLCD_write(0x06, ( worm_tail[2] >> 8));
				clcd_test.CLCD_write(0x07, ( worm_tail[2] & 0xFF));
				clcd_test.CLCD_write(0x08, ((239 - worm_tail[3]) >> 8));
				clcd_test.CLCD_write(0x09, ((239 - worm_tail[3]) & 0xFF));
				clcd_test.CLCD_write_cmd(0x22);
				
				clcd_test.CLCD_master_write_start();
				for(int loop = 0; loop < (5*5); loop++){
					clcd_test.CLCD_write_fw(LightGrey);
				}
				clcd_test.CLCD_master_write_stop();*/
				clcd_test.CLCD_boxsize(worm_tail[0],(320 - worm_tail[1]),worm_tail[2],(240 - worm_tail[3]),LightGrey);
			}
			wait(delay);
		} while (!game_over);
		if(game_over == 1)
		{
			return 1;
		}
		else
		{
			return 0;
		}
}

void setup()
{
	pc.printf("hello and welcome to this test of the serial\n");
	choice = 0;
//	pc.baud(921600);
	srand(z);
  spi_test.format(8,0);
  spi_test.frequency(1000000);
	for(int u = 0; u<=3; u++)
	{
		for(int v = 0; v<=4; v++)
		{
			worm_high_score[u][v] = 0x30;
		}
	}

	for(int i = 0; i<=(number_of_lines-1); i++){
			
			int horishift = rand();
			while (horishift > 320) {
				if(horishift >= 320) {
					horishift = horishift/10;
				}
			}
			int vertshift = rand();
			while (vertshift>190) {
				if(vertshift >= 190) {
					vertshift = vertshift/10;
				}
			}

			for(int j = 0; j<=3; j++){

			if(i == 0){
				if(j == 0){coords[i][j] = 0;}
				else if(j == 1){coords[i][j] = 320-(shapesize);}
				else if(j == 2){coords[i][j] = 10;}
				else if(j == 3){coords[i][j] = 190-(shapesize);}
			} else if(i==1){
				if(j == 0){coords[i][j] = 0;}
				else if(j == 1){coords[i][j] = 320-(shapesize);}
				else if(j == 2){coords[i][j] = 190-(shapesize);}
				else if(j == 3){coords[i][j] = 10;}
			} else if(i==2){
				if(j == 0){coords[i][j] = 320-(shapesize);}
				else if(j == 1){coords[i][j] = 0;}
				else if(j == 2){coords[i][j] = 10;}
				else if(j == 3){coords[i][j] = 190-(shapesize);}
			} else if(i==3){
				if(j == 0){coords[i][j] = 320-(shapesize);}
				else if(j == 1){coords[i][j] = 0;}
				else if(j == 2){coords[i][j] = 190-(shapesize);}
				else if(j == 3){coords[i][j] = 10;}
			} else {
				if(j == 0){coords[i][j] = (coords[0][j]+horishift);}
				else if(j == 1){coords[i][j] = (coords[0][j]-horishift-(shapesize));}
				else if(j == 2){coords[i][j] = coords[0][j]+vertshift;}
				else if(j == 3){coords[i][j] = (coords[0][j]-vertshift-(shapesize));}
			}
		}
	}

	for(int i = 0; i<=(number_of_lines-1); i++){
		int setup_direction[2];
		setup_direction[0]= rand();
		setup_direction[1]= rand();
		for(int k = 0; k<=1; k++){
			int value = setup_direction[k];
			int leftovers = value%2;
			if (leftovers == 1){
				setup_direction[k] = 1;
			}else {
				setup_direction[k] = 0;
			}
		}
				
		if(i == 0){
			hori_ctrl[i] = 1;
			vert_ctrl[i] = 1;
			colour[i] = 0;
		} else if(i == 1){
			hori_ctrl[i] = 0;
			vert_ctrl[i] = 1;
			colour[i] = 0;
		} else if(i == 2){
			hori_ctrl[i] = 1;
			vert_ctrl[i] = 0;
			colour[i] = 0;
		} else if(i == 3){
			hori_ctrl[i] = 0;
			vert_ctrl[i] = 0;
			colour[i] = 0;
		} else {
			hori_ctrl[i] = setup_direction[0];
			vert_ctrl[i] = setup_direction[1];

		}
	}
	
	for(int i = 0; i<=(number_of_lines-1); i++){
		randomcolour[i] = rand();
	}
	for(int i = 0; i<=(number_of_lines-1); i++){
		colourchoice[i] = rand();
	}
	
	for(int i = 0; i<=(number_of_lines-1); i++){
		colour[i] = colourchoice[i]%65536;
	}

	direction_randomiser();
	//clcd_test.CLCD_bitmap(231, 110, 8, 74, (unsigned short *)pressData);
	//wait(5);
	main_menu();

}

// Demo 1 Screen initialisation
void apDEMO1_init(void)
{
	// Set background to black
	clcd_test.CLCD_fillscreen(Black);
	// Draw EXIT button image
  clcd_test.CLCD_bitmapsize (268, 319, 185, 239, (unsigned short *)exitData);

  // Draw the three sliders
  clcd_test.CLCD_bitmapsize (60, 235, 110, 137, (unsigned short *)slideData);

  clcd_test.CLCD_bitmapsize (60, 235, 160, 187, (unsigned short *)slideData);

  clcd_test.CLCD_bitmapsize (60, 235, 210, 237, (unsigned short *)slideData);

  // Draw next icon
  clcd_test.CLCD_bitmapsize (275, 310, 119, 156, (unsigned short *)nextData);
}

// Demo 1 test
void apDEMO1_run(void)
{
	unsigned int loop, done, ucol, ncar, carno;
	unsigned int rpos, gpos, bpos, pend, penx, peny, nx, col, ncol;

    // Screen Initialisation
    apDEMO1_init();
    
	// Set default sliders
	done  = FALSE;
	ucol  = TRUE;
	ncar  = FALSE;
	carno = 0;
	rpos  = 64;
	gpos  = 64;
	bpos  = 64;
	penx  = 64 + 86;
	movesliderhori(rpos + 86, 112, 0xF800, penx);
	movesliderhori(gpos + 86, 162, 0x07E0, penx);
	movesliderhori(bpos + 86, 212, 0x001F, penx);

	do
	{
		// Read touchscreen
		pend = touchscreen.TSC_READXY(&penx, &peny);

		// Done
		if (pend && (penx > 238) && (peny > 200))
		    done = TRUE;

		// Red cursor
		if (pend && (penx > 76) && (penx < 224) && (peny > 95) && (peny < 135))
		{
			nx = (penx < 214) ? penx : 213;
			nx = (nx > 86) ? nx : 87;
			movesliderhori(rpos + 86, 112, 0xF800, nx);
			rpos = nx - 86;
			ucol = TRUE;
		}
		// Green cursor
		if (pend && (penx > 76) && (penx < 224) && (peny > 147) && (peny < 185))
		{
			nx = (penx < 214) ? penx : 213;
			nx = (nx > 86) ? nx : 87;
			movesliderhori(gpos + 86, 162, 0x07E0, nx);
			gpos = nx - 86;
			ucol = TRUE;
		}
		// Blue cursor
		if (pend && (penx > 76) && (penx < 224) && (peny > 195) && (peny < 235))
		{
			nx = (penx < 214) ? penx : 213;
			nx = (nx > 86) ? nx : 87;
			movesliderhori(bpos + 86, 212, 0x001F, nx);
			bpos = nx - 86;
			ucol = TRUE;
		}

		// New car
		if (pend && (penx > 275) && (penx < 310) && (peny > 119) && (peny < 156))
		{
			// Clear the old car
            clcd_test.CLCD_boxsize(0, 319, 0, 110, 0x0000);
			// Next car
			carno = (carno == 2) ? 0 : carno + 1;
			ncar = TRUE;
		}

	    // Draw the new car colour
		if (ucol || ncar)
		{
			// Calculate car colour
			ncol = ((rpos << 9) & 0xF800) | ((gpos << 4) & 0x07E0) | ((bpos >> 2) & 0x001F);

			// Draw car
			if (carno == 0)
			{
				clcd_test.CLCD_setwindowsize (28, 271, 5, 86);
				clcd_test.CLCD_master_write_start();
				for(loop = 0; loop < (244*82); loop++)
				{
					col = (car1Data[loop] & 0xFFFF);
					if (col == 0xF800)
						col = ncol;
					clcd_test.CLCD_write_fw(col);
				}
				clcd_test.CLCD_master_write_stop();
			}
			else if (carno == 1)
			{
				clcd_test.CLCD_setwindowsize (36, 263, 0, 100);
				clcd_test.CLCD_master_write_start();
				for(loop = 0; loop < (228*101); loop++)
				{
					col = (car2Data[loop] & 0xFFFF);
					if (col == 0xF800)
						col = ncol;
					clcd_test.CLCD_write_fw(col);
				}
				clcd_test.CLCD_master_write_stop();
			}
			else
			{
				clcd_test.CLCD_setwindowsize (26, 277, 0, 99);
				clcd_test.CLCD_master_write_start();
				for(loop = 0; loop < (251*100); loop++)
				{
					col = (car3Data[loop] & 0xFFFF);
					if (col == 0xF800)
						col = ncol;
					clcd_test.CLCD_write_fw(col);
				}
				clcd_test.CLCD_master_write_stop();
			}
			if (ncar)
				wait_ms(1000);

			ucol = FALSE;
			ncar = FALSE;
		}
	} while (!done);

	// Change EXIT image
	clcd_test.CLCD_setwindowsize (268, 319, 185, 239);
	clcd_test.CLCD_master_write_start();
	for(loop = 0; loop < (52*54); loop++)
	{
		col = (exitData[loop] & 0xFFFF);
		if(col == 0xFFFF)
    		clcd_test.CLCD_write_fw(0x07E0);
		else
    		clcd_test.CLCD_write_fw(col);
	}
    clcd_test.CLCD_master_write_stop();

	wait_ms(1000);
}

// Demo 2 Screen initialisation
void apDEMO2_init(void)
{
    unsigned int loop;
    
	// Set background to black
    clcd_test.CLCD_fillscreen (Black);

	// Draw EXIT button image
    clcd_test.CLCD_bitmapsize (268, 319, 185, 239, (unsigned short *)exitData);

    // Draw the slider horizontal
    clcd_test.CLCD_bitmapsize (60, 235, 178, 205, (unsigned short *)slideData);

    // Draw the slider vertical
    clcd_test.CLCD_bitmapsize (5, 32, 10, 186, (unsigned short *)slidevData);

	// Draw the wav window
	for (loop = 50; loop < 250; loop++)
	{
        clcd_test.CLCD_putpixelcolor(loop,  49, White);
        clcd_test.CLCD_putpixelcolor(loop, 150, White);
	}
	for (loop = 50; loop < 150; loop++)
	{
        clcd_test.CLCD_putpixelcolor(50, loop, White);
        clcd_test.CLCD_putpixelcolor(250, loop, White);
	}
}

// Demo 2 test
void apDEMO2_run(void)
{
	unsigned int loop, done, uwav, wavh, wavv, uvol, uspk, spkmd, timer, hop;
	unsigned int hpos, vpos, pend, penx, peny, nx, ny, col, x;
	int          y;
    unsigned char din;

	// AACI CODEC init
	din = audio_test.AACI_INIT();

    // Read and check the I2C chip ID and revision
    din = audio_test.AUDIO_I2C_READ(AAIC_I2C_CRID, AAIC_I2C_ADDR);
    if ((din & 0xF8) != 0xE0)
    {
        pc.printf("ERROR: AACI ID:0x%02X\n", din);
    }
    else
    {
        pc.printf("AACI ID:0x%02X\n", din);
    }

    // Initialise screen for demo
    apDEMO2_init();
    
	// Set default sliders
	done  = FALSE;
	uwav  = TRUE;
	uvol  = TRUE;
	uspk  = TRUE;
	spkmd = TRUE;
	pend  = TRUE;
	wavv  = 64;
	wavh  = 64;
	hpos  = 64;
	vpos  = 64;
	penx  = 64 + 86;
    
	movesliderhori(hpos + 86, 180, 0x07E0, hpos + 86);
	moveslidervert(7, vpos + 25, 0x07E0, vpos + 25);

	do
	{
		// Output sinewave until pen is released
		if (!pend && spkmd)
		{
			// Wav hop size
			hop = (42 + hpos) >> 4;

			// Clear TSC Interrupt
            NVIC_ClearPendingIRQ(TSC_IRQn);
			//*NVIC_CLRPEND0 |= (1 << NVIC_TSC);
			do
			{
				// Wait for TX FIFO not to be full then write left and right channels
				timer = AACI_TIMEOUT;
				while ((MPS2_AAIC_I2S->STATUS & I2S_STATUS_TXFull_Msk) && timer)
					timer--;

				// Left then right audio out
				MPS2_AAIC_I2S->TXBUF = sinewave[loop];

				// Next sine value
				loop = loop + hop;
				if (loop > 359)
					loop = loop - 359;

			} while (!NVIC_GetPendingIRQ(TSC_IRQn));
		}

		// Check TSC interrupt
		if (NVIC_GetPendingIRQ(TSC_IRQn))
		{
            // Clear interrupt
			NVIC_ClearPendingIRQ(TSC_IRQn);
            
			touchscreen.TSC_WRITE(0x0B, 0x01, TSC_I2C_ADDR);
		}

		// Read touchscreen
		pend = touchscreen.TSC_READXY(&penx, &peny);

		// Done
		if (pend && (penx > 238) && (peny > 200))
		    done = TRUE;

		// Select Speaker or Mic mode
		if (pend && (penx > 275) && (penx < 320) && (peny > 40) && (peny < 140))
		{
			if(peny < 90)
				spkmd = TRUE;
			else
				spkmd = FALSE;
			uspk = TRUE;
		}

		// Draw slider horizontal (spos = 1 to 127)
		if (pend && (penx > 76) && (penx < 224) && (peny > 153) && (peny < 230))
		{
			nx = (penx < 214) ? penx : 213;
			nx = (nx > 86) ? nx : 87;
			movesliderhori(hpos + 86, 180, 0x07E0, nx);
			hpos = nx - 86;
			uwav = TRUE;
		}

		// Draw slider vertical (spos = 1 to 127)
		if (pend && (penx > 0) && (penx < 37) && (peny > 0) && (peny < 196))
		{
			ny = (peny < 150) ? peny : 150;
			ny = (ny > 24) ? ny : 24;
			moveslidervert(7, vpos + 25, 0x07E0, ny + 2);
			vpos = ny - 23;
			uvol = TRUE;
			uwav = TRUE;
		}

	    // Update volume
		if (uvol)
		{
			loop = 0xFF - (vpos >> 1);
			audio_test.AUDIO_I2C_WRITE(0x20, loop, AAIC_I2C_ADDR);
			audio_test.AUDIO_I2C_WRITE(0x21, loop, AAIC_I2C_ADDR);
		}

	    // Draw wave
		if (uwav)
		{
			// Clear the old wav
			for (x = 51; x < 249; x++)
			{
				// Calculate X
				hop = (x * (wavh + 64)) / 32;
				while (hop >= 360)
					hop = hop - 360;

				// Calculate Y
				y = (sinewave[hop] / 0xFFFF) ^ 0xFFFF8000;
				y = y * (128 - wavv) * 200;

				// Draw wav
				clcd_test.CLCD_putpixelcolor(x,  100 + (y >> 24), Black);
			}

			// New wav h/v value
			wavh = hpos;
			wavv = vpos;

			// Draw the new wav
			for (x = 51; x < 249; x++)
			{
				// Calculate X
				hop = (x * (wavh + 64)) / 32;
				while (hop >= 360)
					hop = hop - 360;

				// Calculate Y
				y = (sinewave[hop] / 0xFFFF) ^ 0xFFFF8000;
				y = y * (128 - wavv) * 200;

				// Draw wav
				clcd_test.CLCD_putpixelcolor(x,  100 + (y >> 24), White);
			}

			if (spkmd)
			{
				// Draw the grids
				for (loop = 51; loop < 249; loop = loop + 5)
					clcd_test.CLCD_putpixelcolor(loop,  100, LightGrey);
				for (loop = 51; loop < 149; loop = loop + 5)
					clcd_test.CLCD_putpixelcolor(150, loop, LightGrey);
			}

			uwav = FALSE;
		}

	    // Draw speaker and mic
		if (uspk)
		{
			// Draw the speaker
			clcd_test.CLCD_setwindowsize (280, 315, 50, 84);
			clcd_test.CLCD_master_write_start();
			for(loop = 0; loop < (35*35); loop++)
			{
				col = (spkrData[loop] & 0xFFFF);
				if ((col == 0xC618) && spkmd)
					col = 0x07E0;
				clcd_test.CLCD_write_fw(col);
			}
			clcd_test.CLCD_master_write_stop();

			// Draw the speaker off
			clcd_test.CLCD_setwindowsize (280, 315, 100, 134);
			clcd_test.CLCD_master_write_start();
			for(loop = 0; loop < (35*35); loop++)
			{
				col = (spkroffData[loop] & 0xFFFF);
				if ((col == 0xC618) && !spkmd)
					col = 0xF800;
				if ((col == 0x8410) && !spkmd)
					col = 0x07E0;
				clcd_test.CLCD_write_fw(col);
			}
			clcd_test.CLCD_master_write_stop();

			uspk = FALSE;
		}

	} while (!done);

	// Change EXIT image
	clcd_test.CLCD_setwindowsize (268, 319, 185, 239);
	clcd_test.CLCD_master_write_start();
	for(loop = 0; loop < (52*54); loop++)
	{
		col = (exitData[loop] & 0xFFFF);
		if(col == 0xFFFF)
    		clcd_test.CLCD_write_fw(0x07E0);
		else
    		clcd_test.CLCD_write_fw(col);
	}
    clcd_test.CLCD_master_write_stop();

	wait_ms(1000);
}
// Demo 3 Screen initialisation
void apDEMO3_init(void)
{
    unsigned int x;
    
	// Set background to black
	clcd_test.CLCD_fillscreen(Black);

	// Draw EXIT button image
    clcd_test.CLCD_bitmapsize (268, 319, 185, 239, (unsigned short *)exitData);

    // Draw the slider
    clcd_test.CLCD_bitmapsize (60, 235, 105, 132, (unsigned short *)slideData);

	// Set the default LED images
	for (x = 0; x < 8; x++)
	{
        clcd_test.CLCD_bitmapsize (40 + (x * 30), 63 + (x * 30), 30, 77, (unsigned short *)ledoffData);
	}
}

// Demo 3 test
void apDEMO3_run(void)
{
	unsigned int loop, done, uled, usw, ledv, swv;
	unsigned int spos, pend, penx, peny, nx, col, x;

    apDEMO3_init();

// Set default sliders
	done = FALSE;
	uled = TRUE;
	usw  = TRUE;
	ledv = 0;
	swv  = 0;
	spos = 64;
	penx = 64 + 86;
	movesliderhori(spos + 86, 107, 0x007E0, penx);
	MPS2_SCC->LEDS = 0x08;

	do
	{
		// Read touchscreen
		pend = touchscreen.TSC_READXY(&penx, &peny);

		// Done
		if (pend && (penx > 238) && (peny > 200))
		    done = TRUE;

		// Led cursor
		if (pend && (penx > 76) && (penx < 224) && (peny > 90) && (peny < 147))
		{
			nx = (penx < 214) ? penx : 213;
			nx = (nx > 86) ? nx : 87;
			movesliderhori(spos + 86, 107, 0x07E0, nx);
			spos = nx - 86;
			uled = TRUE;
		}

	    // Draw the new leds
		if (uled)
		{
			// Clear the old LED image
            clcd_test.CLCD_bitmapsize (40 + ((7 - ledv) * 30), 63 + ((7 - ledv) * 30), 30, 77, (unsigned short *)ledoffData);

			// Calculate LED position
			if (spos > 112)
				ledv = 0;
			else if (spos > 96)
				ledv = 1;
			else if (spos > 80)
				ledv = 2;
			else if (spos > 64)
				ledv = 3;
			else if (spos > 48)
				ledv = 4;
			else if (spos > 32)
				ledv = 5;
			else if (spos > 16)
				ledv = 6;
			else
				ledv = 7;

			// Set the actual LED
			MPS2_SCC->LEDS = 0x01 << ledv;

			// Set the LED image
            clcd_test.CLCD_bitmapsize (40 + ((7 - ledv) * 30), 63 + ((7 - ledv) * 30), 30, 77, (unsigned short *)ledonData);

			uled = FALSE;
		}

	    // Update switches
		if (usw | (swv != (MPS2_SCC->SWITCHES & 0xFF)))
		{
	        // Read the switch value
	        swv = MPS2_SCC->SWITCHES & 0xFF;

	        // Draw the switch
            clcd_test.CLCD_bitmapsize (60, 235, 146, 235, (unsigned short *)switchData);

	        // Draw the switches
	        for (x = 0; x < 8; x++)
	        {
	        	if (swv & (0x80 >> x))
	        	{
					clcd_test.CLCD_boxsize(68 + (x * 21), 77 + (x * 21), 189, 195, 0x001F);
	        	}
	        	else
	        	{
					clcd_test.CLCD_boxsize(68 + (x * 21), 77 + (x * 21), 180, 186, 0xF800);
	        	}
	        }
	    	usw  = FALSE;
		}
	} while (!done);

	// Change EXIT image
	clcd_test.CLCD_setwindowsize (268, 319, 185, 239);
	clcd_test.CLCD_master_write_start();
	for(loop = 0; loop < (52*54); loop++)
	{
		col = (exitData[loop] & 0xFFFF);
		if(col == 0xFFFF)
    		clcd_test.CLCD_write_fw(0x07E0);
		else
    		clcd_test.CLCD_write_fw(col);
	}
    clcd_test.CLCD_master_write_stop();

	wait_ms(1000);
}

unsigned int ethernet_loopback_test(void)
{
	unsigned int index;
  unsigned int txfifo_inf;
  unsigned int pktsize;
    
  index = 0;
  pktsize = TX_PKT_SIZE;
	
	txfifo_inf = testing.Ethernet_intf();
  //pc.printf("TX_FIFO_INF: %#08x\n", txfifo_inf);

	if((txfifo_inf & 0xFFFF) >= pktsize)
	{
		// Send single packet.
 //   testing.write(txpkt, pktsize);
		testing.transmission(txpkt, pktsize);
	} else {
		printf("Insufficient tx fifo space for packet size %d\n",pktsize);
		return 1;
	}
	
  wait_ms(5);

  // Receive all that's available in Rx DATA Fifo.
  if(testing.reception((unsigned int *)rxpkt, &index))
	{
		pc.printf("Packet receive failed.\n");
    return 1;
  } /*else {
		testing.read((unsigned int *)rxpkt, &index);
	}*/

  if(compare_buf(rxpkt, txpkt, TX_PKT_SIZE)) 
	{
		pc.printf("Sent and received packets do not match.\n");
		return 1;
	}
	
	return 0;
}

// Basic equivalent of memcmp. Compares two buffers up to size of len.
unsigned int compare_buf(unsigned char *buf1, unsigned char *buf2, int len)
{
    int i;
    unsigned char *tmpbuf1, *tmpbuf2;

    tmpbuf1 = buf1;
    tmpbuf2 = buf2;

    if(len < 0 || len > 0x1000) {
        pc.printf("Buffer length invalid or larger than 4KB.\n");
        return 1;
    }

    if(!buf1 || !buf2) {
        pc.printf("Invalid buffer pointers for comparison.\n");
        return 1;
    }

    for(i = 0; i < len; i++) {
        if(*tmpbuf1 != *tmpbuf2) {
            return 1;
        }
        tmpbuf1++;
        tmpbuf2++;
    }
    return 0;
}

