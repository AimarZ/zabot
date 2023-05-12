using UnityEngine;
using UnityEngine.Perception.Randomization.Randomizers;

public class YRotationRandomizerTag : RandomizerTag
{
    private Vector3 originalRotation;
    private Vector3 originalScale;

    private void Start()
    {
        originalRotation = transform.eulerAngles;
        originalScale = transform.localScale;
    }

    public void SetYRotation(float yRotation)
    {
        transform.eulerAngles = new Vector3(originalRotation.x, yRotation, originalRotation.z);
    }

    public void SetScale(float scalex, float scaley, float scalez)
    {
        transform.localScale = new Vector3(originalScale.x*scalex,originalScale.y*scaley,originalScale.z*scalez);
    }
}
