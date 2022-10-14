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

        private void Awake()
        {
            HostIPManager.SetHostIP(HostIP);
        }

        private void Start()
        {
            pub = GetComponent<Publisher>();

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
                toolTarget.StartTracking();
                researchMode.StartToolTracking();
                Invoke("ActivateSensorSwitch", 2.0f);
            }
            else
            {
                sensorSwitch.IsEnabled = false;
                pub.StopDataPublisher();
                toolTarget.StopTracking();
                researchMode.StopToolTracking();
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