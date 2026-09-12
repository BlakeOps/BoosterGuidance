
using KSP.Localization;
using ToolbarControl_NS;
using UnityEngine;
using KSP_Log;


namespace BoosterGuidance
{
    [KSPAddon(KSPAddon.Startup.Instantly, true)]
    public class InitLog : MonoBehaviour
    {
        public static KSP_Log.Log Log;

        public static void SetLogLevel(int i)
        {
            Log.SetLevel((Log.LEVEL)i);
        }

        protected void Awake()
        {
            // Always log at INFO level: runtime messages (guidance enable/disable,
            // logging start/stop) are the only way to diagnose user flight issues
            Log = new KSP_Log.Log("BoosterGuidance", KSP_Log.Log.LEVEL.INFO);
        }
    }

    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public class RegisterToolbar : MonoBehaviour
    {
        private void Start()
        {
            ToolbarControl.RegisterMod(MainWindow.MODID, MainWindow.MODNAME);
        }
    }
}
