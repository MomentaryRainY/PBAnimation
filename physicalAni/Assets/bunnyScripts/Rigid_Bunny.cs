using UnityEngine;
using System.Collections;
using System;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	bool convergence = false;

    float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.998f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.4f;                 // for vertically collision
	float friction = 0.2f;						// for horizontal collision

	Vector3[] vertices;


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		if (convergence)
		{
			v = new Vector3();
			w = new Vector3();
			return;
		}

		Vector3 hitVertices = new Vector3();
		bool collision = false;
		int collisionNumber = 0;
        Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
        for (int i = 0; i < vertices.Length; i++)
		{
            Vector3 Rri = R.MultiplyVector(vertices[i]); 
            float distance = Vector3.Dot((Rri + transform.position - P), N);
			if (distance < 0)
			{
				Vector3 Vi1 = v + Vector3.Cross(w, Rri);
				if (Vector3.Dot(Vi1, N) < 0)
				{
                    collision = true;
                    hitVertices += vertices[i];
					collisionNumber++;
                }
			}
        }

		if (!collision) return;

		Vector3 ris = hitVertices / collisionNumber;
		Vector3 Rris = R.MultiplyVector(ris);

		Vector3 Vi = v + Vector3.Cross(w, Rris);
		if (Vi.magnitude < 0.8f) 
		{ 
			convergence = true;
		}
		Vector3 Vn = Vector3.Dot(Vi, N) * N;
		Vector3 Vt = Vi - Vn;
		Vector3 VnN = -restitution * Vn;
		Vector3 VtN = Mathf.Max(0, 1 - friction * (1 + restitution) * Vn.magnitude / Vt.magnitude) * Vt;
		Vector3 ViN = VnN + VtN;

		Matrix4x4 RriStar = Get_Cross_Matrix(Rris);
		Matrix4x4 I_inv = (R * I_ref * R.transpose).inverse;
		Matrix4x4 K = MatrixSub(MatrixMulFloat(Matrix4x4.identity, 1 / mass), RriStar * I_inv * RriStar);
		Vector3 J = K.inverse.MultiplyVector(ViN - Vi);

		v += 1 / mass * J;
		w += I_inv.MultiplyVector(Vector3.Cross(Rris, J));
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			transform.eulerAngles = new Vector3(50, 0, 0);
			launched=false;
			convergence = false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			w = new Vector3 (1, 1, 0);
			launched=true;
		}

		if (launched)
		{
            // Part I: Update velocities
            v *= linear_decay;
			w *= angular_decay;

            Vector3 g = new Vector3(0, -9.8f, 0);
            v += dt * g;

            // Part II: Collision Impulse
            Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
            Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

            // Part III: Update position & orientation
            //Update linear status
            Vector3 x = transform.position;
            x = x + v * dt;

			//Update angular status
            Quaternion q = transform.rotation;
			Vector3 dw = 0.5f * dt * w;
			Quaternion qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);

            q = Add(q, q * qw);

            // Part IV: Assign to the object
            transform.position = x;
            transform.rotation = q;
        }

	}

	Matrix4x4 FMulMatrix(float a, Matrix4x4 b)
	{
        Matrix4x4 A = b;
        A[0, 0] *= a;
        A[0, 1] *= a;
        A[0, 2] *= a;
        A[1, 0] *= a;
        A[1, 1] *= a;
        A[1, 2] *= a;
        A[2, 0] *= a;
        A[2, 1] *= a;
        A[2, 2] *= a;
        A[3, 3] *= a;

		return A;
    }

	Quaternion Add(Quaternion a, Quaternion b)
	{
		return new Quaternion(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
	}

	Matrix4x4 MatrixMulFloat(Matrix4x4 a, float b)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				a[i, j] *= b;
			}
		}
		return a;
	}

	Matrix4x4 MatrixSub(Matrix4x4 a, Matrix4x4 b)
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				a[i, j] -= b[i, j];
			}
		}
		return a;
	}
}
