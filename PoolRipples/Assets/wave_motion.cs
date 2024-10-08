using UnityEngine;
using System.Collections;
using static UnityEditor.PlayerSettings;

public class wave_motion : MonoBehaviour 
{
	int size 		= 100;
	float rate 		= 0.005f;
	float gamma		= 0.004f;
	float damping 	= 0.996f;
	float[,] 	old_h;
	float[,]	low_h;
	float[,]	vh;
	float[,]	b;

	bool [,]	cg_mask;
	float[,]	cg_p;
	float[,]	cg_r;
	float[,]	cg_Ap;
	bool 	tag=true;

	Vector3 	cube_v = Vector3.zero;
	Vector3 	cube_w = Vector3.zero;


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();

		Vector3[] X=new Vector3[size*size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			X[i*size+j].x=i*0.1f-size*0.05f;
			X[i*size+j].y=0;
			X[i*size+j].z=j*0.1f-size*0.05f;
		}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i=0; i<size-1; i++)
		for (int j=0; j<size-1; j++)
		{
			T[index*6+0]=(i+0)*size+(j+0);
			T[index*6+1]=(i+0)*size+(j+1);
			T[index*6+2]=(i+1)*size+(j+1);
			T[index*6+3]=(i+0)*size+(j+0);
			T[index*6+4]=(i+1)*size+(j+1);
			T[index*6+5]=(i+1)*size+(j+0);
			index++;
		}
		mesh.vertices  = X;
		mesh.triangles = T;
		mesh.RecalculateNormals ();

		low_h 	= new float[size,size];
		old_h 	= new float[size,size];
		vh 	  	= new float[size,size];
		b 	  	= new float[size,size];

		cg_mask	= new bool [size,size];
		cg_p 	= new float[size,size];
		cg_r 	= new float[size,size];
		cg_Ap 	= new float[size,size];

		for (int i=0; i<size; i++)
		for (int j=0; j<size; j++) 
		{
			low_h[i,j]=99999;
			old_h[i,j]=0;
			vh[i,j]=0;
		}
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			Ax[i,j]=0;
			if(i!=0)		Ax[i,j]-=x[i-1,j]-x[i,j];
			if(i!=size-1)	Ax[i,j]-=x[i+1,j]-x[i,j];
			if(j!=0)		Ax[i,j]-=x[i,j-1]-x[i,j];
			if(j!=size-1)	Ax[i,j]-=x[i,j+1]-x[i,j];
		}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret=0;
		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			ret+=x[i,j]*y[i,j];
		}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for(int i=li; i<=ui; i++)
		for(int j=lj; j<=uj; j++)
		if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
		{
			cg_p[i,j]=cg_r[i,j]=b[i,j]-cg_r[i,j];
		}

		float rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for(int k=0; k<128; k++)
		{
			if(rk_norm<1e-10f)	break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha=rk_norm/Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				x[i,j]   +=alpha*cg_p[i,j];
				cg_r[i,j]-=alpha*cg_Ap[i,j];
			}

			float _rk_norm=Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta=_rk_norm/rk_norm;
			rk_norm=_rk_norm;

			for(int i=li; i<=ui; i++)
			for(int j=lj; j<=uj; j++)
			if(i>=0 && j>=0 && i<size && j<size && mask[i,j])
			{
				cg_p[i,j]=cg_r[i,j]+beta*cg_p[i,j];
			}
		}

	}

	void Shallow_Wave(float[,] old_h, float[,] h, float [,] new_h)
	{
        //Step 1:
        //TODO: Compute new_h based on the shallow wave model.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                new_h[i, j] = h[i, j] + damping * (h[i, j] - old_h[i, j]);
                if (i > 0) new_h[i, j] += rate * (h[i - 1, j] - h[i, j]);
                if (i < size - 1) new_h[i, j] += rate * (h[i + 1, j] - h[i, j]);
                if (j > 0) new_h[i, j] += rate * (h[i, j - 1] - h[i, j]);
                if (j < size - 1) new_h[i, j] += rate * (h[i, j + 1] - h[i, j]);
            }
        }

		//Step 2: Block->Water coupling
		//TODO: for block 1, calculate low_h.
		Vector3 blockPosition = GameObject.Find("Block").GetComponent<MeshFilter>().transform.position;
        
        Bounds blockBound = GameObject.Find("Block").GetComponent<Renderer>().bounds;
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				float dist = 999999;
				blockBound.IntersectRay(new Ray(new Vector3(i * 0.1f - size * 0.05f, 0, j * 0.1f - size * 0.05f), Vector3.up), out dist);
				low_h[i, j] = dist;
			}
		}

        for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
                if (low_h[i, j] < new_h[i, j])
				{
                    b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
					//vh[i, j] = low_h[i, j];
                    cg_mask[i, j] = true;
				} else
				{
					b[i, j] = 0;
                    cg_mask[i, j] = false;
                }

            }	
		}
        int grid_x = (int)(blockPosition.x * 10 + size * 0.5f);
        int grid_z = (int)(blockPosition.z * 10 + size * 0.5f);
        int li = grid_x - 6;
        int ui = grid_x + 6;
        int lj = grid_z - 6;
        int uj = grid_z + 6;
        Conjugate_Gradient(cg_mask, b, vh, li, ui, lj, uj);

        Vector3 cubePosition = GameObject.Find("Cube").GetComponent<MeshFilter>().transform.position;
		Bounds cubeBound = GameObject.Find("Cube").GetComponent<Renderer>().bounds;
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				float dist = 99999;
				cubeBound.IntersectRay(new Ray(new Vector3(i * 0.1f - size * 0.05f, 0, j * 0.1f - size * 0.05f), Vector3.up), out dist);
				low_h[i, j] = dist;
			}
		}
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                if (low_h[i, j] < new_h[i, j])
                {
                    b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
					//vh[i, j] = low_h[i, j];
                    cg_mask[i, j] = true;
                }
                else
                {
                    b[i, j] = 0;
                    cg_mask[i, j] = false;
                }
            }
        }

        int grid_x1 = (int)(cubePosition.x * 10 + size * 0.5f);
        int grid_z1 = (int)(cubePosition.z * 10 + size * 0.5f);
        int li1 = grid_x1 - 6;
        int ui1 = grid_x1 + 6;
        int lj1 = grid_z1 - 6;
        int uj1 = grid_z1 + 6;
        //TODO: then set up b and cg_mask for conjugate gradient.
        Conjugate_Gradient(cg_mask, b, vh, li1, ui1, lj1, uj1);
        //TODO: Solve the Poisson equation to obtain vh (virtual height).

        //TODO: Diminish vh.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                vh[i, j] *= gamma;
            }
        }

        //TODO: Update new_h by vh.
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                if (i > 0) new_h[i, j] += rate * (vh[i - 1, j] - vh[i, j]);
                if (i < size - 1) new_h[i, j] += rate * (vh[i + 1, j] - vh[i, j]);
                if (j > 0) new_h[i, j] += rate * (vh[i, j - 1] - vh[i, j]);
                if (j < size - 1) new_h[i, j] += rate * (vh[i, j + 1] - vh[i, j]);
            }
        }

        //Step 3
        //TODO: old_h <- h; h <- new_h;
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                old_h[i, j] = h[i, j];
                h[i, j] = new_h[i, j];
            }
        }

		//Step 4: Water->Block coupling.
		//More TODO here.
		float mass = 10f;
        float dt = 0.004f;
		Vector3 torque = Vector3.zero;
        Vector3 force = new Vector3(0, -9.8f * mass, 0);
        for (int i = li1; i < ui1; i++)
		{
			for(int j = lj1; j < uj1; j++)
			{
				if (i > 0 && i < size && j > 0 && j < size)
				{
                    Vector3 bottom = new Vector3(i * 0.1f - size * 0.05f, -1, j * 0.1f - size * 0.05f);
                    //Debug.Log(i * 0.1f - size * 0.05f);
                    float dist;
					blockBound.IntersectRay(new Ray(bottom, Vector3.up), out dist);
					//if (dist != 0) Debug.Log(dist);
					Vector3 up = new Vector3(i * 0.1f - size * 0.05f, dist - 10, j * 0.1f - size * 0.05f);

					Vector3 Rri = GameObject.Find("Cube").GetComponent<MeshFilter>().transform.InverseTransformPoint(up) - blockPosition;
					
					if (vh[i, j] != 0)
					{
						Vector3 f = new Vector3(0, vh[i, j], 0) * 4.0f;
                        force += f;
						torque += Vector3.Cross(Rri, f);
					}
                }
			}
		}
		cube_v *= 0.98f;
		cube_v += dt * force / mass;
		cubePosition += cube_v * dt;
		GameObject.Find("Cube").transform.position = cubePosition;

        cube_w *= 0.98f;
		cube_w += dt * torque / mass / 100f;
		Quaternion q = GameObject.Find("Cube").transform.rotation;
		Quaternion wq = new Quaternion(0.5f * dt * cube_w.x, 0.5f * dt * cube_w.y, 0.5f * dt * cube_w.z, 0) * q;
		Quaternion cube_q = new Quaternion();
		cube_q.x = q.x + wq.x;
        cube_q.y = q.y + wq.y;
        cube_q.z = q.z + wq.z;
        cube_q.w = 0;
		GameObject.Find("Cube").transform.rotation = cube_q;
    }
	

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X    = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h     = new float[size, size];

		//TODO: Load X.y into h.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				h[i, j] = X[i * size + j].y;
			}
		}

		if (Input.GetKeyDown ("r")) 
		{
            //TODO: Add random water.
            int ri = Random.Range(0, size);
            int rj = Random.Range(0, size);
            float rh = Random.Range(0.1f, 1.0f);
            h[ri, rj] += rh;

            int neighbors = 0;
            if (ri > 0) neighbors++;
            if (ri < size - 1) neighbors++;
            if (rj > 0) neighbors++;
            if (rj < size - 1) neighbors++;

            rh /= neighbors;
            if (ri > 0) h[ri - 1, rj] -= rh;
            if (ri < size - 1) h[ri + 1, rj] -= rh;
            if (rj > 0) h[ri, rj - 1] -= rh;
            if (rj < size - 1) h[ri, rj + 1] -= rh;

        }
	
		for(int l=0; l<8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

        //TODO: Store h back into X.y and recalculate normal.

        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                X[i * size + j].y = h[i, j];
            }
        }
		mesh.vertices = X;
		mesh.RecalculateNormals();
    }

}
