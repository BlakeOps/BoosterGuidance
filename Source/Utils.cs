using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static BoosterGuidance.InitLog;

namespace BoosterGuidance
{
    public static class Utils
    {
        static System.IO.StreamWriter actual = null;
        static System.IO.StreamWriter free = null;
        static System.IO.StreamWriter unset = null;
        static System.IO.StreamWriter simuate = null;
        static System.IO.StreamWriter manual = null;
        public enum LogType { none, actual, free, unset, simuate, manual };

        const string LOGDIR = "Logs/BoosterGuidance/";

        static bool loggingActive = false;
        static public bool LoggingActive { get { return loggingActive; } }

        // Name of the current session's log files (without directory/extension),
        // e.g. "Size3LargeTank_20261002_042020". Null when not logging.
        static public string SessionLogName { get; private set; }

        static public void StartLogging(string shipName)
        {
            if (loggingActive)
            {
                InitLog.Log.Info("StartLogging(" + shipName + ") called while already active - restarting");
                EndLogging();
            }
            string LogsPath = KSPUtil.ApplicationRootPath + "Logs";
            if (!Directory.Exists(LogsPath))
                Directory.CreateDirectory(LogsPath);
            if (!Directory.Exists(KSPUtil.ApplicationRootPath + LOGDIR))
                Directory.CreateDirectory(KSPUtil.ApplicationRootPath + LOGDIR);

            // Session-scoped filenames (flight-logging-v2 LOG-3): a reflight of the
            // same vessel name must never truncate the previous flight's log.
            string safeName = shipName;
            foreach (char c in System.IO.Path.GetInvalidFileNameChars())
                safeName = safeName.Replace(c, '_');
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string sessionBase = LOGDIR + safeName + "_" + stamp;
            SessionLogName = safeName + "_" + stamp;

            actual = new System.IO.StreamWriter(sessionBase + ".Actual.dat");
            free = new System.IO.StreamWriter(sessionBase + ".Free.dat");
            unset = new System.IO.StreamWriter(sessionBase + ".Simulate.Unset.dat");
            simuate = new System.IO.StreamWriter(sessionBase + ".Simulate.dat");
            manual = new System.IO.StreamWriter(sessionBase + ".manual.dat");
            // Flush every line so the log survives a crash/alt-F4 instead of
            // sitting in the StreamWriter buffer until Close
            actual.AutoFlush = true;
            free.AutoFlush = true;
            unset.AutoFlush = true;
            simuate.AutoFlush = true;
            manual.AutoFlush = true;
            loggingActive = true;
            InitLog.Log.Info("StartLogging: " + SessionLogName);
        }

        static public void EndLogging()
        {
            if (loggingActive)
            {
                InitLog.Log.Info("EndLogging");
                actual.Close();
                free.Close();
                unset.Close();
                simuate.Close();
                manual.Close();
                loggingActive = false;
                SessionLogName = null;
            }
        }
        static public void Log(LogType logtype, string str)
        {
            if (loggingActive)
            {
                switch (logtype)
                {
                    case LogType.actual: actual.WriteLine(str); break;
                    case LogType.free: free.WriteLine(str); break;
                    case LogType.unset: unset.WriteLine(str); break;
                    case LogType.simuate: simuate.WriteLine(str); break;
                    case LogType.manual: manual.WriteLine(str); break;
                }
            }
        }
    }
}
