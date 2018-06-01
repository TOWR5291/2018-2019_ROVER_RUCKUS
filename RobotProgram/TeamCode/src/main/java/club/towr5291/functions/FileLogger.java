package club.towr5291.functions;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Date;

/**
 * Created by T6810SM on 9/9/2015.
 */
public class FileLogger {

    private final static int BUFFER_SIZE = 10;
    private String[] buffer = new String[BUFFER_SIZE];
    private int index = 0;
    private FileWriter writer;
    private String filename;
    private ElapsedTime elapsedTime;
    private String filenamePrefix = "";
    private boolean isOpen = false;
    private int numFilesToSave = 50;
    private int debugLevel;
    private boolean enableLogd = false;

    public FileLogger(ElapsedTime elapsedTime) {
        open();
        this.elapsedTime = elapsedTime;
        setDebugLevel(1);
        setLogdEnabled(false);
    }

    public FileLogger(ElapsedTime elapsedTime, int debug) {
        open();
        this.elapsedTime = elapsedTime;
        setDebugLevel(debug);
        setLogdEnabled(false);
    }

    public FileLogger(ElapsedTime elapsedTime, int debug, boolean enabled) {
        open();
        this.elapsedTime = elapsedTime;
        setDebugLevel(debug);
        setLogdEnabled(enabled);
    }

    public int getDebugLevel() {
        return this.debugLevel;
    }

    public void setDebugLevel(int debug) {
        this.debugLevel = debug;
        writeEvent("SETTING:", "Debug Level Set " + this.debugLevel);
    }

    public boolean getLogdEnabled() {
        return this.enableLogd;
    }

    public void setLogdEnabled(boolean enabled) {
        this.enableLogd = enabled;
        writeEvent("SETTING:", "logd enabled? " + this.enableLogd);

    }

    public String getFilename() {
        return this.filename;
    }

    public String getFilenamePrefix() { return this.filenamePrefix; }

    public boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        if (Environment.MEDIA_MOUNTED.equals(state)) {
            return true;
        }
        return false;
    }

    public String getStoragePath() {
        return Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS + "/Logs").toString();
    }

    public File getStorageFullPath(String fileName) {
        // Get the directory for the user's public docs directory.
        File file = new File(getStoragePath(), fileName);
        return file;
    }

    public void open() {
        String outFile = "";
        if (!isOpen)
            try {
                long tm = System.currentTimeMillis();
                this.filenamePrefix = ""+tm;
                String fileName = "rlog"+tm+".txt";
                if (isExternalStorageWritable()) {
                    deleteOldLogs();
                    File out = getStorageFullPath(fileName);
                    writer = new FileWriter(out.getAbsolutePath(),true);
                    if (out != null) {
                        this.filename = out.toString();
                        System.out.println("Opened file: "+out.toString());
                        isOpen = true;
                    }
                }
                else {
                    System.out.println("External Storage is not writable...");
                }
            } catch( Exception ex) {
                Log.e("Err","Caught Exception opening file: "+outFile+", ex: "+ex);
            }
    }

    public void saveBitmap(Bitmap bm, String filename)
    {
        String fileName = this.filenamePrefix + filename;
        try {
            if (isExternalStorageWritable()) {
                File out = getStorageFullPath(fileName);
                FileOutputStream stream = new FileOutputStream(out);
                bm.compress(Bitmap.CompressFormat.JPEG, 100, stream);
                stream.flush();
                stream.close();
            }
            else {
                System.out.println("External Storage is not writable...");
            }
        } catch( Exception ex) {
            Log.e("Err","Caught Exception opening file: "+fileName+", ex: "+ex);
        }
    }

    public void close() {
        try {
            writeBuffer();
            writer.flush();
            writer.close();
        } catch( Exception ex) {
            System.out.println("Caught Exception closing file: "+ex);
        }
        isOpen = false;
    }

    public synchronized void writeEvent(String event, String desc) {
        if (this.enableLogd) {
            if (event.length() > 23) {
                event = event.substring(0, 22);
            }
            Log.d(event.toUpperCase(), desc);
        }
        if (isOpen)
            this.write(this.elapsedTime.toString() + "," + System.currentTimeMillis() + "," + Thread.currentThread().getId() + "," + event + "," + desc);
    }

    public synchronized void writeEvent(int debug, String event, String desc) {
        if ( this.debugLevel >= debug ) {
            if (this.enableLogd) {
                if (event.length() > 23) {
                    event = event.substring(0, 22);
                }
                Log.d(event.toUpperCase(), desc);
            }
            if (isOpen)
                this.write(this.elapsedTime.toString() + "," + System.currentTimeMillis() + "," + Thread.currentThread().getId() + "," + event + "," + desc);
        }
    }

    public synchronized void write(String line) {
        if (isOpen) {
            if (index == BUFFER_SIZE) {
                writeBuffer();
            }
            buffer[index] = line;
            index++;
        }
    }

    private void writeBuffer() {
        for (int i=0; i<buffer.length; i++) {
            try {
                if (writer != null) {
                    writer.flush();
                    if (buffer[i] != null)
                        writer.write(buffer[i]);
                    writer.write('\r');
                    writer.write('\n');
                }
            } catch( Exception ex) {
                System.out.println("Caught Exception writing buffer["+i+"] value: "+ex);
            }
        }
        index = 0;
    }

    private void deleteOldLogs() {
        String path = getStoragePath();
        File directory = new File(path);
        File[] files = directory.listFiles();
        //Log.d("Files", "Size: "+ files.length);

        Arrays.sort( files, new Comparator()
        {
            public int compare(Object o1, Object o2) {

                if (((File)o1).lastModified() > ((File)o2).lastModified()) {
                    return -1;
                } else if (((File)o1).lastModified() < ((File)o2).lastModified()) {
                    return +1;
                } else {
                    return 0;
                }
            }

        });

        for (int i = 0; i < files.length; i++) {
            if (i <= numFilesToSave) {
                Log.d("Files", "Keeping FileName: " + i + " , " + files[i].getName() + " Modified " + files[i].lastModified());
            } else {
                Log.d("Files", "Deleting FileName: " + i + " , " + files[i].getName() + " Modified " + files[i].lastModified());
                files[i].delete();
            }
        }
    }

}
