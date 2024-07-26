package org.frcteam6941.utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;

public class CSVReader {
    public static String[][] readAsMatrix(String filePath) {
        List<String[]> rowList = new ArrayList<>();
        try (BufferedReader br = new BufferedReader(
                new FileReader(new File(Filesystem.getDeployDirectory(), filePath + ".csv")))) {
            String line;
            while ((line = br.readLine()) != null) {
                String[] lineItems = line.split(",");
                rowList.add(lineItems);
            }
            br.close();
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }

        String[][] matrix = new String[rowList.size()][5];
        for (int i = 0; i < rowList.size(); i++) {
            String[] row = rowList.get(i);
            matrix[i] = row;
        }
        return matrix;
    }
}
