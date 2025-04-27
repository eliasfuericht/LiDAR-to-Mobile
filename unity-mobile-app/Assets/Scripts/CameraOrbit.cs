using UnityEngine;

public class MobileOrbitCamera : MonoBehaviour
{
    public float rotationSpeed = 0.2f;
    public float zoomSpeed = 0.05f;
    public float distance = 20.0f;
    public float minDistance = 0.1f;
    public float maxDistance = 100.0f;

    private Vector2 lastTouchPosition;
    private bool isDragging = false;
    private float yaw = 0.0f;
    private float pitch = 0.0f;

    void Update()
    {
        if (Input.touchCount == 1)
        {
            Touch touch = Input.GetTouch(0);

            if (touch.phase == TouchPhase.Began)
            {
                isDragging = true;
                lastTouchPosition = touch.position;
            }
            else if (touch.phase == TouchPhase.Moved && isDragging)
            {
                Vector2 delta = touch.position - lastTouchPosition;
                yaw += delta.x * rotationSpeed;
                pitch -= delta.y * rotationSpeed;
                pitch = Mathf.Clamp(pitch, -80f, 80f);

                lastTouchPosition = touch.position;
            }
            else if (touch.phase == TouchPhase.Ended || touch.phase == TouchPhase.Canceled)
            {
                isDragging = false;
            }
        }
        else if (Input.touchCount == 2)
        {
            Touch touch0 = Input.GetTouch(0);
            Touch touch1 = Input.GetTouch(1);

            Vector2 touch0PrevPos = touch0.position - touch0.deltaPosition;
            Vector2 touch1PrevPos = touch1.position - touch1.deltaPosition;

            float prevMagnitude = (touch0PrevPos - touch1PrevPos).magnitude;
            float currentMagnitude = (touch0.position - touch1.position).magnitude;

            float difference = currentMagnitude - prevMagnitude;

            distance -= difference * zoomSpeed;
            distance = Mathf.Clamp(distance, minDistance, maxDistance);
        }

        Quaternion rotation = Quaternion.Euler(pitch, yaw, 0);
        Vector3 direction = rotation * Vector3.forward;
        transform.position = -direction * distance;
        transform.LookAt(Vector3.zero);
    }
}
