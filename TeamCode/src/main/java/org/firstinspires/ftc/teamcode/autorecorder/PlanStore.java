package org.firstinspires.ftc.teamcode.autorecorder;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/*
 * One text file per plan in a folder. On a robot use new PlanStore(new File(AppUtil.ROOT_FOLDER, "recorded")),
 * which is /sdcard/recorded/ on the Control Hub.
 */
public final class PlanStore {
    private final File folder;
    private final Map<Plan, File> sources = new HashMap<>();   // the file each loaded plan came from

    public PlanStore(File folder) {
        this.folder = folder;
    }

    private File fileFor(String name) {
        return new File(folder, name.replaceAll("[^A-Za-z0-9]+", "_") + ".txt");
    }

    /** True if a plan with this name is already saved (saving replaces it). */
    public boolean exists(String name) {
        return fileFor(name).exists();
    }

    /** Returns false if it couldn't be written. An old plan of the same name survives a failed save. */
    public boolean save(Plan plan) {
        File f = fileFor(plan.name);
        File temp = new File(folder, f.getName() + ".tmp");
        try {
            if (!folder.exists()) folder.mkdirs();
            try (FileWriter w = new FileWriter(temp, false)) {
                w.write("// name " + plan.name + "\n");
                if (plan.start != null) w.write("// start " + plan.start + "\n");
                int e = 0;
                for (double[] p : plan.path) {
                    while (e < plan.events.size() && plan.events.get(e).time <= p[0]) write(w, plan.events.get(e++));
                    w.write(String.format(Locale.US, "p %.2f %.2f %.2f %.4f\n", p[0], p[1], p[2], p[3]));
                }
                while (e < plan.events.size()) write(w, plan.events.get(e++));
            }
            if (f.exists() && !f.delete()) return false;
            return temp.renameTo(f);
        } catch (Exception ex) {
            return false;
        }
    }

    private static void write(FileWriter w, Plan.Event e) throws IOException {
        w.write(String.format(Locale.US, "e %.2f %s %s", e.time, e.name, e.value).trim() + "\n");
    }

    /** Every saved plan, sorted by file name. Files it can't read are left out. */
    public List<Plan> loadAll() {
        List<Plan> plans = new ArrayList<>();
        sources.clear();
        File[] files = folder.listFiles();
        if (files == null) return plans;
        Arrays.sort(files);
        for (File f : files) {
            if (!f.getName().endsWith(".txt")) continue;
            Plan p = load(f);
            if (p == null) continue;
            plans.add(p);
            sources.put(p, f);
        }
        return plans;
    }

    /** The plan in this file, tidied the way it will replay, or null if it isn't a plan. */
    public static Plan load(File f) {
        Plan plan = new Plan(null);
        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
            for (String line = r.readLine(); line != null; line = r.readLine()) {
                line = line.trim();
                String[] t = line.split("\\s+");
                if (line.startsWith("// name ")) plan.name = line.substring(8).trim();
                else if (line.startsWith("// start ")) plan.start = t[2];
                else if (t[0].equals("p")) plan.path.add(new double[]{Double.parseDouble(t[1]),
                        Double.parseDouble(t[2]), Double.parseDouble(t[3]), Double.parseDouble(t[4])});
                else if (t[0].equals("e")) plan.addEvent(Double.parseDouble(t[1]), t[2], t.length > 3 ? t[3] : "");
            }
        } catch (Exception e) {
            return null;
        }
        if (plan.name == null || plan.path.isEmpty()) return null;
        plan.tidy();
        return plan;
    }

    /**
     * Takes a plan from loadAll() out of the list by moving its file to a deleted/ folder, so a mistake can be
     * undone by moving it back. Returns false if it couldn't.
     */
    public boolean delete(Plan plan) {
        File source = sources.get(plan);
        if (source == null) return false;
        File trash = new File(folder, "deleted");
        if (!trash.exists() && !trash.mkdirs()) return false;
        File target = new File(trash, source.getName());
        if (target.exists() && !target.delete()) return false;
        if (!source.renameTo(target)) return false;
        sources.remove(plan);
        return true;
    }
}
