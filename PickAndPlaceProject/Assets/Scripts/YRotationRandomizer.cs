using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.Perception.Randomization.Parameters;
using UnityEngine.Perception.Randomization.Randomizers;
using UnityEngine.Perception.Randomization.Samplers;


[Serializable]
[AddRandomizerMenu("Perception/Y Rotation Randomizer")]
public class YRotationRandomizer : Randomizer
{
    public FloatParameter rotationRange = new FloatParameter { value = new UniformSampler(0f, 360f)}; // in range (0, 1)
    public FloatParameter scaleRange = new FloatParameter { value = new UniformSampler(0.5f, 2f)}; // in range (1, 3)
    public bool uniformScale = true;

    protected override void OnIterationStart()
    {
        IEnumerable<YRotationRandomizerTag> tags = tagManager.Query<YRotationRandomizerTag>();
        foreach (YRotationRandomizerTag tag in tags)
        {
            float yRotation = rotationRange.Sample();
            float scalex = scaleRange.Sample();
            float scaley = scaleRange.Sample();
            float scalez = scaleRange.Sample();

            if (uniformScale){
                scaley = scalex;
                scalez = scalex;
            }

            // sets rotation
            tag.SetYRotation(yRotation);

            tag.SetScale(scalex, scaley, scalez);
        }
    }
}
