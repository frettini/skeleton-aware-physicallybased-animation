using System.IO;
using System.Text;
using UnityEngine;

public class DataToText
{
    private const int MAX_SIZE = 20000;
    private float[,] m_data;
    private float m_numFeatures;
    private int lineInd = 0;

    public DataToText(int numFeatures)
    {
        m_numFeatures = numFeatures;
        m_data = new float[MAX_SIZE, numFeatures];
    }

    public void AddLine(float[] line)
    {
        for (int i = 0; i < m_numFeatures; i++)
        {
            m_data[lineInd,i] = line[i];
        }
        lineInd++;
    }

    public void WriteToText(string fileName)
    {
        Debug.Log("Start Saving Data!");
        //before your loop
        var csv = new StringBuilder();

        for (int i = 0; i < lineInd; i++)
        {
            string line = "";

            for (int j = 0; j < m_numFeatures; j++)
            {
                line += m_data[i, j].ToString();
                if(j == m_numFeatures - 1)
                {
                    //line += '\n';
                }
                else
                {
                    line += ',';
                }
            }
            csv.AppendLine(line);
        }

        //after your loop
        File.WriteAllText(Application.dataPath + fileName, csv.ToString());
        Debug.Log("Finished Saving Data!");
    }
}
