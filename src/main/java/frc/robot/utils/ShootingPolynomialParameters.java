package frc.robot.utils;

import com.team254.lib.util.PolynomialRegression;

public class ShootingPolynomialParameters {
	double[][] distance = {
			{1.1, 23},//20240804
			{1.75, 42},//20240804
			{2.23, 54},//20240804
			{2.54, 55},//20240804
			{2.76, 59},//20240804
	   		{1.07, 17.8},//20240805
	   		{1.66, 33.8},//20240805
	   		{2.33, 38.8},//20240805
	   		{2.44, 51.6},//20240805
	   		{2.90, 56.8},//20240805
	   		{3.17, 57.7},//20240805
	   		{3.41, 60.9},//20240805
	  		{3.74, 63.9},//20240805
			{1.01, 15.9},//20240806
			{1.84, 36.8},//20240806
			{2.57, 52.8},//20240806
			{3.28, 57.7},//20240806
			{ 3.65, 64.7 }//20240806
		};

	PolynomialRegression disToDeg = new PolynomialRegression(distance, 5);
	 
	public double getAngle(double distance) {
		return disToDeg.predict(distance);
	}
}
