/*
 * Eis Case
 * Copyright (c) 2022 Lone Dynamics Corporation. All rights reserved.
 *
 * required hardware:
 *  - 4 x M3 x 16mm countersunk bolts
 *  - 4 x M3 nuts
 *
 */

$fn = 100;

board_width = 40;
board_thickness = 1.5;
board_length = 80;
board_height = 1.6;
board_spacing = 2;

wall = 1.5;

top_height = 9;
bottom_height = 8;

//ldn_board();

translate([0,0,15])
	ldn_case_top();

//translate([0,0,-15])
//	ldn_case_bottom();

module ldn_board() {
	
	difference() {
		color([0,0.5,0])
			roundedcube(board_width,board_length,board_thickness, 2);
		translate([5, 5, -1]) cylinder(d=3.2, h=10);
		translate([5, 75, -1]) cylinder(d=3.2, h=10);
		translate([35, 5, -1]) cylinder(d=3.2, h=10);
		translate([35, 75, -1]) cylinder(d=3.2, h=10);
	}	
	
}

module ldn_case_top() {
	
	difference() {
				
		color([0.5,0.5,0.5])
			roundedcube(board_width+(wall*2),board_length+(wall*2),top_height,2.5);

		// cutouts
			
		translate([2,9,-2])
			roundedcube(board_width-1.25,board_length-15,9.25,2.5);

		translate([9,3,-2])
			roundedcube(board_width-15,board_length-3,9.25,2.5);
		
		translate([wall, wall, 0]) {
	
			// HDMI
			translate([30,19-(15.5/2),-1]) cube([30,15.5,6.2+1]);
		
			// USBA
			translate([30,43-(15/2),-1]) cube([30,15,7.2+1.05]);

			// USBC
			translate([30,64.25-(9.5/2),-1]) cube([30,9.5,3.5+1]);
		
			// SD
			translate([20-(15/2),70,-1]) cube([15,30,2+1]);

			// PMODA
			translate([20-(16/2),-10,-2]) cube([16,30,5.5+1.15]);

			// LED vent
			// translate([-5,25,4]) cube([30,1.5,1.5]);
			// translate([-5,20,4]) cube([30,1.5,1.5]);
			// translate([-5,15,4]) cube([30,1.5,1.5]);
				
			// bolt holes
			translate([5, 5, -21]) cylinder(d=3.5, h=40);
			translate([5, 75, -21]) cylinder(d=3.5, h=40);
			translate([35, 5, -20]) cylinder(d=3.5, h=40);
			translate([35, 75, -21]) cylinder(d=3.5, h=40);

			// flush mount bolt holes
			translate([5, 5, top_height-1]) cylinder(d=5, h=4);
			translate([5, 75, top_height-1]) cylinder(d=5, h=4);
			translate([35, 5, top_height-1]) cylinder(d=5, h=4);
			translate([35, 75, top_height-1]) cylinder(d=5, h=4);

			// eis text
			rotate(270)
				translate([-21,20-3,top_height-0.8])
					linear_extrude(1)
						text("E   I   S", size=6, halign="center",
							font="Ubuntu:style=Bold");

		}
		
	}	
}

module ldn_case_bottom() {
	
	difference() {
		color([0.5,0.5,0.5])
			roundedcube(board_width+(wall*2),board_length+(wall*2),bottom_height,2.5);
		
		// cutouts
		translate([3,10,1.5])
			roundedcube(board_width-3,board_length-17,10,2.5);
				
		translate([10.5,2.5,1.5])
			roundedcube(board_width-17.5,board_length-2,10,2.5);

		translate([wall, wall, 0]) {
			
		// board cutout
		translate([-0.25,-0.25,bottom_height-board_height])
			roundedcube(board_width+0.5,board_length+0.5,board_height+1,2);

		// bolt holes
		translate([5, 5, -11]) cylinder(d=3.2, h=25);
		translate([5, 75, -11]) cylinder(d=3.2, h=25);
		translate([35, 5, -11]) cylinder(d=3.2, h=25);
		translate([35, 75, -11]) cylinder(d=3.2, h=25);

		// nut holes
		translate([5, 5, -1]) cylinder(d=7, h=2.5+2, $fn=6);
		translate([5, 75, -1]) cylinder(d=7, h=2.5+2, $fn=6);
		translate([35, 5, -1]) cylinder(d=7, h=2.5+2, $fn=6);
		translate([35, 75, -1]) cylinder(d=7, h=2.5+2, $fn=6);

		}
		
	}	
}

// https://gist.github.com/tinkerology/ae257c5340a33ee2f149ff3ae97d9826
module roundedcube(xx, yy, height, radius)
{
    translate([0,0,height/2])
    hull()
    {
        translate([radius,radius,0])
        cylinder(height,radius,radius,true);

        translate([xx-radius,radius,0])
        cylinder(height,radius,radius,true);

        translate([xx-radius,yy-radius,0])
        cylinder(height,radius,radius,true);

        translate([radius,yy-radius,0])
        cylinder(height,radius,radius,true);
    }
}
