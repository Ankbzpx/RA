using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class Block : MonoBehaviour {

    [SerializeField]
    public int blockId;

    Rigidbody rb;
    Renderer render;
    Color originalColor;

    private void Awake()
    {
        render = GetComponent<Renderer>();
        rb = GetComponent<Rigidbody>();
        originalColor = render.material.color;
        //Register the block, save its orignal position and rotation
        SceneController.RegisterBlock(blockId, this, transform.position, transform.rotation);
    }

    void Update()
    {
        if (blockId == HighlightBlock.highlightedBlockId)
        {
            if(render.material.color != Color.red)
                render.material.color = Color.red;
        }
        else
        {
            if(render.material.color != originalColor)
                render.material.color = originalColor;
        }
    }


    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.tag == "Ground")
        {
            SceneController.SetNonExtractable(blockId);
            SceneController.BlockDropGround(blockId);
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.tag == "Ground")
        {
            SceneController.SetNonExtractable(blockId);
            SceneController.BlockDropGround(blockId);
        }
    }

    public int GetId()
    {
        return blockId;
    }
}
