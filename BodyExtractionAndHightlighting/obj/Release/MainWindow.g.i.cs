﻿#pragma checksum "..\..\MainWindow.xaml" "{406ea660-64cf-4c82-b6f0-42d48172a799}" "8CA6A36D96F85433920862D235316AD8"
//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:4.0.30319.34209
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

using System;
using System.Diagnostics;
using System.Windows;
using System.Windows.Automation;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Markup;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Effects;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Media.TextFormatting;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Shell;


namespace BodyExtractionAndHightlighting {
    
    
    /// <summary>
    /// MainWindow
    /// </summary>
    public partial class MainWindow : System.Windows.Window, System.Windows.Markup.IComponentConnector {
        
        
        #line 39 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.RadioButton fullHD;
        
        #line default
        #line hidden
        
        
        #line 40 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.RadioButton nonFullHD;
        
        #line default
        #line hidden
        
        
        #line 42 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Slider userTransparency;
        
        #line default
        #line hidden
        
        
        #line 44 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.RadioButton BlackBG;
        
        #line default
        #line hidden
        
        
        #line 45 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.RadioButton WhiteBG;
        
        #line default
        #line hidden
        
        
        #line 46 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.RadioButton CustomBG;
        
        #line default
        #line hidden
        
        
        #line 47 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.CheckBox checkBoxExtendArm;
        
        #line default
        #line hidden
        
        
        #line 50 "..\..\MainWindow.xaml"
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1823:AvoidUnusedPrivateFields")]
        internal System.Windows.Controls.Image imageCombi;
        
        #line default
        #line hidden
        
        private bool _contentLoaded;
        
        /// <summary>
        /// InitializeComponent
        /// </summary>
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        public void InitializeComponent() {
            if (_contentLoaded) {
                return;
            }
            _contentLoaded = true;
            System.Uri resourceLocater = new System.Uri("/BodyExtractionAndHightlighting;component/mainwindow.xaml", System.UriKind.Relative);
            
            #line 1 "..\..\MainWindow.xaml"
            System.Windows.Application.LoadComponent(this, resourceLocater);
            
            #line default
            #line hidden
        }
        
        [System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [System.CodeDom.Compiler.GeneratedCodeAttribute("PresentationBuildTasks", "4.0.0.0")]
        [System.ComponentModel.EditorBrowsableAttribute(System.ComponentModel.EditorBrowsableState.Never)]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Design", "CA1033:InterfaceMethodsShouldBeCallableByChildTypes")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        [System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1800:DoNotCastUnnecessarily")]
        void System.Windows.Markup.IComponentConnector.Connect(int connectionId, object target) {
            switch (connectionId)
            {
            case 1:
            
            #line 9 "..\..\MainWindow.xaml"
            ((BodyExtractionAndHightlighting.MainWindow)(target)).MouseRightButtonDown += new System.Windows.Input.MouseButtonEventHandler(this.Window_MouseRightButtonDown);
            
            #line default
            #line hidden
            return;
            case 2:
            this.fullHD = ((System.Windows.Controls.RadioButton)(target));
            
            #line 39 "..\..\MainWindow.xaml"
            this.fullHD.Checked += new System.Windows.RoutedEventHandler(this.fullHD_Checked);
            
            #line default
            #line hidden
            return;
            case 3:
            this.nonFullHD = ((System.Windows.Controls.RadioButton)(target));
            
            #line 40 "..\..\MainWindow.xaml"
            this.nonFullHD.Checked += new System.Windows.RoutedEventHandler(this.nonFullHD_Checked);
            
            #line default
            #line hidden
            return;
            case 4:
            this.userTransparency = ((System.Windows.Controls.Slider)(target));
            return;
            case 5:
            this.BlackBG = ((System.Windows.Controls.RadioButton)(target));
            
            #line 44 "..\..\MainWindow.xaml"
            this.BlackBG.Checked += new System.Windows.RoutedEventHandler(this.BlackBG_Checked);
            
            #line default
            #line hidden
            return;
            case 6:
            this.WhiteBG = ((System.Windows.Controls.RadioButton)(target));
            
            #line 45 "..\..\MainWindow.xaml"
            this.WhiteBG.Checked += new System.Windows.RoutedEventHandler(this.WhiteBG_Checked);
            
            #line default
            #line hidden
            return;
            case 7:
            this.CustomBG = ((System.Windows.Controls.RadioButton)(target));
            
            #line 46 "..\..\MainWindow.xaml"
            this.CustomBG.Checked += new System.Windows.RoutedEventHandler(this.CustomBG_Checked);
            
            #line default
            #line hidden
            return;
            case 8:
            this.checkBoxExtendArm = ((System.Windows.Controls.CheckBox)(target));
            
            #line 47 "..\..\MainWindow.xaml"
            this.checkBoxExtendArm.Checked += new System.Windows.RoutedEventHandler(this.checkBoxExtendArm_Checked);
            
            #line default
            #line hidden
            
            #line 47 "..\..\MainWindow.xaml"
            this.checkBoxExtendArm.Unchecked += new System.Windows.RoutedEventHandler(this.checkBoxExtendArm_Unchecked);
            
            #line default
            #line hidden
            return;
            case 9:
            this.imageCombi = ((System.Windows.Controls.Image)(target));
            return;
            }
            this._contentLoaded = true;
        }
    }
}

