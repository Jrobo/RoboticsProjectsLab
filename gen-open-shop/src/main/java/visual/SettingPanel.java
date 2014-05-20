package visual;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

/**
 * Created by Tatiyana Domanova on 5/20/14.
 */
public class SettingPanel extends JPanel{

    private final static Dimension size = new Dimension(300,500);

    public SettingPanel() {
        setLayout(new BorderLayout());

        JPanel topPanel = new JPanel();
        topPanel.setLayout(new FlowLayout());
        JLabel topLabel = new JLabel("Number of Machines");
        JTextField topText = new JTextField();
        JButton topButton = new JButton("Apply");
        topButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                /* Clear table
                *  Set number of columns*/
            }
        });

        topPanel.add(topLabel);
        topPanel.add(topText);
        topPanel.add(topButton);
        topPanel.setVisible(true);


        setVisible(true);
    }

}
