package visual;

import javax.swing.*;
import java.awt.*;

/**
 * Created by Tatiyana Domanova on 5/20/14.
 */
public class Application extends JFrame {

    private final static Dimension size = new Dimension(700,500);

    /*
    * Need to set Problem to Schedule Manager
    * Need to set new population to EvolutionManager
    * */

    public Application () {
        setTitle("Genetic Algorithm for Open-Shop Scheduling");
        setSize(size);
//        setLayout(new BorderLayout());

        getContentPane().add(new SettingPanel());
//        add(new ViewPanel(), BorderLayout.CENTER);

        setDefaultCloseOperation(EXIT_ON_CLOSE);
    }


    public static void main(String arg[]) {
        setLookAndFeel();
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                Application app = new Application();
                app.setVisible(true);
            }
        });
    }

    private static void setLookAndFeel() {
        try {
            UIManager.setLookAndFeel("com.sun.java.swing.plaf.nimbus.NimbusLookAndFeel");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
