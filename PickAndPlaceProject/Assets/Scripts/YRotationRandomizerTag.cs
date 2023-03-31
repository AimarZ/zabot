using UnityEngine;
using UnityEngine.Perception.Randomization.Randomizers;

public class YRotationRandomizerTag : RandomizerTag
{
    private Vector3 originalRotation;

    private void Start()
    {
        originalRotation = transform.eulerAngles;
    }

    public void SetYRotation(float yRotation)
    {
        transform.eulerAngles = new Vector3(originalRotation.x, yRotation, originalRotation.z);
    }

    public void SetScale(float scalex, float scaley, float scalez)
    {
        transform.localScale = new Vector3(1.3f*scalex,1.2f*scaley,1.3f*scalez);
    }
}
