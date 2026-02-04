package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
public class FileController {

    public static void write(String filename, List<Double> data) {
        String dataString = data.toString();
        dataString = dataString.substring(1, dataString.length() - 1);

        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, dataString);
    }

    public static List<Double> read(String filename) {
        List<Double> data = new ArrayList<> ();

        File closeAuto = AppUtil.getInstance().getSettingsFile(filename);
        String[] types = ReadWriteFile.readFile(closeAuto).trim().split(",");
        for (String type : types) {
            if (!type.isEmpty()) {
                data.add(Double.parseDouble(type.trim()));
            }
        }
        return data;
    }

}
