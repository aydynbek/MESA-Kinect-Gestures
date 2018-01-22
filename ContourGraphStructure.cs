using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Samples.Kinect.BodyBasics
{
    class ContourGraphStructure
    {
        private int[] verticees = null;
        private int[] adjacencyMap = null;
        private byte[] orientationMap = null;

        public ContourGraphStructure(int[] verticees, int[] adjacencyMap, byte[] orientationMap)
        {
            this.verticees = verticees;
            this.adjacencyMap = adjacencyMap;
            this.orientationMap = orientationMap;
        }

    }
}
