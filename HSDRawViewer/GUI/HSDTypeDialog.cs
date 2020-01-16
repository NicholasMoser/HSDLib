﻿using HSDRaw;
using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Windows.Forms;

namespace HSDRawViewer.GUI
{
    public partial class HSDTypeDialog : Form
    {
        public Type HSDAccessorType { get => (Type)comboBoxType.SelectedItem; }

        public HSDTypeDialog()
        {
            InitializeComponent();

            List<Type> types = (from domainAssembly in AppDomain.CurrentDomain.GetAssemblies()
                                     from assemblyType in domainAssembly.GetTypes()
                                     where typeof(HSDAccessor).IsAssignableFrom(assemblyType)
                                     select assemblyType).ToList();

            foreach(var v in types)
            {
                comboBoxType.Items.Add(v);
            }

        }

        private void buttonOkay_Click(object sender, EventArgs e)
        {
            DialogResult = DialogResult.OK;
            Close();
        }

        private void comboBoxType_SelectedIndexChanged(object sender, EventArgs e)
        {

        }
    }
}
