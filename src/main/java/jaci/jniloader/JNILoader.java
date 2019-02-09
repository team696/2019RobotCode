package jaci.jniloader;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;

/**
 * The JNILoader is a structure used to help with loading JNI and other Native Libraries from System Path,
 * relative file, or embedded jar resource.
 *
 * System Path is given highest priority, followed by `platform_string`/, followed by any user-defined
 * paths.
 *
 * @author Jaci Brunning
 * https://github.com/JacisNonsense/JNILoader
 */
public class JNILoader {

    private final String libraryName;
    private boolean loaded;
    private List<String> testPaths;

    /**
     * Create a new JNI Loader
     * @param libraryName The name of the library, for example: libtest.so is 'test'
     */
    public JNILoader(String libraryName) {
        this.libraryName = libraryName;
        this.testPaths = new ArrayList<>();
        testPaths.add(getPlatformString() + "/" + getSharedLibraryFile());
    }

    /**
     * Attempt to load the native file. This function is re-entrant.
     */
    public void load() {
        if (!loaded) {
            doLoad();
            loaded = true;
        }
    }

    /**
     * Add a path to check for the Native Library. This will check both relative to the working
     * directory on the filesystem (highest priority), followed by an attempted extraction of an
     * embedded jar resource.
     *
     * @param path The path to check for the Native Library, e.g: myfolder/ will check in
     *             `working_dir`/myfolder/ and classpath:/myfolder/
     */
    public void addPath(String path) {
        this.testPaths.add(path);
    }

    /**
     * Get the platform string, generated from the OS and Architecture of the current system.
     * @return The platform string, e.g. linuxx86-64 or osxx86
     */
    public static String getPlatformString() {
        return getNormalizedOS() + getNormalizedArchitecture();
    }

    /**
     * Get the name of the shared library file based on the current platform.
     * @return The name of the shared library file (e.g. libmylib.so for linux, libmylib.dylib
     *          for mac, mylib.dll for windows).
     */
    public String getSharedLibraryFile() {
        String os = getNormalizedOS();
        String prefix = os.equals("windows") ? "" : "lib";
        String suffix = os.equals("windows") ? ".dll" : os.equals("osx") ? ".dylib" : ".so";
        return prefix + libraryName + suffix;
    }

    /**
     * Get the normalized OS string from the os.name system property.
     * @return The normalized OS string (windows, mac, linux).
     */
    public static String getNormalizedOS() {
        String os = System.getProperty("os.name");
        if (os.startsWith("Win"))
            return "windows";
        else if (os.toLowerCase().contains("mac"))
            return "osx";
        else if (os.toLowerCase().contains("linux"))
            return "linux";
        return os.toLowerCase();
    }

    /**
     * Get the normalized Architecture string from the os.arch system property.
     * @return The normalized Architecture string (x86-64, x86)
     */
    public static String getNormalizedArchitecture() {
        String arch = System.getProperty("os.arch");
        if (arch.equals("amd64") || arch.equals("x86_64"))
            return "x86-64";
        else if (arch.equals("i386") || arch.equals("x86"))
            return "x86";
        return arch.toLowerCase();
    }

    private void doLoad() {
        try {
            System.loadLibrary(libraryName);
        } catch (UnsatisfiedLinkError ex) {
            // Does not exist on system path - fallback to other options
            boolean trying = true;
            for (String path : testPaths) {
                try {
                    tryLoadRelative(path);
                    trying = false;
                } catch (UnsatisfiedLinkError ex2) {
                    try {
                        tryLoadJarResource(path);
                        trying = false;
                    } catch (IOException | UnsatisfiedLinkError ex3) { }
                }
            }

            if (trying)
                throw new UnsatisfiedLinkError(libraryName + " could not be found at any location!");
        }
    }

    public void tryLoadRelative(String path) throws UnsatisfiedLinkError {
        System.load(new File(path).getAbsolutePath());
    }

    public void tryLoadJarResource(String path) throws IOException, UnsatisfiedLinkError {
        File f = extractJarResource(path);
        try {
            System.load(f.getAbsolutePath());
        } finally {
            if (f.exists())
                f.delete();
        }
    }

    private File extractJarResource(String path) throws IOException {
        File output = File.createTempFile("jni_" + libraryName, getSharedLibraryFile());
        output.deleteOnExit();

        InputStream is = JNILoader.class.getResourceAsStream("/" + path);
        if (is == null)
            throw new IOException("Resource /" + path + " does not exist!");
        OutputStream os = new FileOutputStream(output);

        byte[] buffer = new byte[2048];
        int readBytes;
        try {
            while ((readBytes = is.read(buffer)) != -1)
                os.write(buffer, 0, readBytes);
        } finally {
            os.close();
            is.close();
        }

        return output;
    }
}
