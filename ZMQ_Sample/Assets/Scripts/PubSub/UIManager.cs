using UnityEngine;
using UnityEngine.UI;
using Microsoft.MixedReality.Toolkit.UI;
using TMPro;

namespace PubSub
{
    public class UIManager : MonoBehaviour
    {
        [SerializeField] private string HostIP;
        [SerializeField] private Interactable sensorSwitch, previewToggle;
        [SerializeField] private TextMeshPro messageBox;
        [SerializeField] private ResearchModeVideoStream researchMode;
        [SerializeField] private IRToolTrack.IRToolController toolTarget;

        private Publisher pub;
        private ReqRep.Client reqClient;

        private void Awake()
        {
            HostIPManager.SetHostIP(HostIP);
        }

        private void Start()
        {
            pub = GetComponent<Publisher>();
            reqClient = GetComponent<ReqRep.Client>();

            sensorSwitch?.OnClick.AddListener(ToggleSensorEvent);
            previewToggle?.OnClick.AddListener(TogglePreview);

            DebugConsole.InitMessageBox(messageBox);
        }

        #region Button Events
        public void ToggleSensorEvent()
        {
            if (sensorSwitch.IsToggled)
            {
                sensorSwitch.IsEnabled = false;
                pub.StartDataPublisher();
                reqClient.StartClient(); 
                DebugConsole.Log("Starting tool tracking");
                toolTarget.StartTracking();
                DebugConsole.Log("Starting sensors");
                researchMode.StartToolTracking();
                Invoke("ActivateSensorSwitch", 2.0f);
            }
            else
            {
                sensorSwitch.IsEnabled = false;
                pub.StopDataPublisher();
                reqClient.StopClient();
                researchMode.StopSensorsEvent();
                toolTarget.StopTracking();
                Invoke("ActivateSensorSwitch", 2.0f);
            }
        }

        private void ActivateSensorSwitch()
        {
            sensorSwitch.IsEnabled = true;
        }

        public void TogglePreview()
        {
            researchMode.TogglePreview();
        }

        #endregion

        public void OnHandAppear()
        {
            foreach (Renderer r in GetComponentsInChildren<Renderer>())
            {
                r.enabled = true;
            }
            foreach (Collider c in GetComponentsInChildren<Collider>())
            {
                c.enabled = true;
            }
        }

        public void OnHandDisappear()
        {
            foreach (Renderer r in GetComponentsInChildren<Renderer>())
            {
                r.enabled = false;
            }
            foreach (Collider c in GetComponentsInChildren<Collider>())
            {
                c.enabled = false;
            }
        }
    }
}