#!/usr/bin/python

import sys

class MDP(object):



    def write_prism_model(self,file_name):
        f=open(file_name,'w')
        f.write('mdp\n \n')
        f.write('module M \n \n')
        f.write('s:[0..'+str(self.n_states-1)+'] init ' + str(self.initial_state) + ';\n \n')
        
        
        
        for i in range(0,self.n_states):
            for j in range(0,self.n_actions):
                current_trans_list=self.transitions[i][j]
                if current_trans_list:
                    trans_string='[' + self.actions[j] + '] s=' + str(i) + ' -> '
                    for trans in current_trans_list:
                        trans_string=trans_string + str(trans[1]) + ":(s'=" + str(trans[0]) + ') + '
                    f.write(trans_string[:-3] + ';\n')    
        
        
        f.write('\nendmodule\n\n')
        
        for i in range(0,self.n_props):
            f.write('label "'+ self.props[i] + '" = ')
            prop_string=''
            for j in range(0,self.n_states):
                if self.prop_map[j][i]:
                   prop_string=prop_string + 's=' + str(j) + ' | '
            f.write(prop_string[:-3] + ';\n')  
        
        f.write('\n')
        
  
        #f.write('label "goal" = ')
        
        #goal_states_string=''
        #for goal_state in self.goal_states:
            #goal_states_string=goal_states_string + 's=' + str(goal_state) + ' | '
            
        #f.write(goal_states_string[:-3] + ';\n\n')
        
        f.write('rewards "time"\n')
        
        for i in range(0,self.n_states):
            for j in range(0,self.n_actions):
                if self.rewards[i][j] != 0:
                    f.write('    [' + self.actions[j] + '] s=' + str(i) + ':' + str(self.rewards[i][j]) + ';\n')
        
        f.write('endrewards\n')
        
        f.close()




class NavMDP(MDP):
    def __init__(self,nav_graph):
        self.read_graph(nav_graph)
        
        
    def read_graph(self,nav_graph):
        f = open(nav_graph, 'r')
        
        self.initial_state=0
        
        
        line=f.readline()
        line=line.split(' ')
        self.n_nodes=int(line[0])
        self.n_edges=int(line[1])
        self.n_states=self.n_nodes+2*self.n_edges+1
        
        self.n_props=self.n_nodes+2
        self.props=[None]*self.n_props
        
        for i in range(0,self.n_props-2):
            self.props[i]='v' + str(i)
            
        self.props[self.n_props-2]='failure'
        self.props[self.n_props-1]='fatal_failure'
        
        
        self.prop_map=[[False]*self.n_props for i in range(self.n_states)]
        
        for i in range(0,self.n_props-2):
            self.prop_map[i][i]=True
            
        for i in range(self.n_nodes,self.n_nodes+self.n_edges):
            self.prop_map[i][self.n_props-2]=True
            
        self.prop_map[self.n_states-1][self.n_props-1]=True
        
        
        
        self.n_actions=self.n_edges+1
        self.actions=[None]*self.n_actions
        self.actions[self.n_actions-1]='recover'
        
        self.transitions=[[False]*self.n_actions for i in range(self.n_states)]
        
        
        self.rewards=[[0]*self.n_actions for i in range(self.n_states)]
        
        
        current_line=0
        for line in f:
            line=line.split()
            from_node=int(line[0])
            to_node=int(line[1])
            suc_rate=float(line[2])
            self.actions[current_line]='goto'+str(from_node) + 'p' + str(to_node)
            self.transitions[from_node][current_line]=[[to_node,suc_rate]]
            if suc_rate<1:
                self.transitions[from_node][current_line].append([self.n_nodes+current_line,round(1-suc_rate,3)])
                i=0
                finished_rec_states=False
                while not finished_rec_states:
                    if not line[3+i+1]=='+':
                        finished_rec_states=True
                    current_trans=line[3+i].split(':')
                    prob=float(current_trans[0])
                    next_node=int(current_trans[1])
                    if i==0:
                        if from_node==next_node:
                            self.transitions[self.n_nodes+current_line][self.n_edges]=[[self.n_nodes+self.n_edges+current_line,prob]]
                        else:
                            self.transitions[self.n_nodes+current_line][self.n_edges]=[[next_node,prob]]
                    else:
                        if from_node==next_node:
                            self.transitions[self.n_nodes+current_line][self.n_edges].append([self.n_nodes+self.n_edges+current_line,prob])
                        else:	
                            self.transitions[self.n_nodes+current_line][self.n_edges].append([next_node,prob])
                    i+=2
                self.rewards[self.n_nodes+current_line][self.n_actions-1]=max(round(float(line[3+i])-float(line[3+i-1]),3),0)  
                self.rewards[from_node][current_line]=float(line[3+i-1])
            else:
                self.rewards[from_node][current_line]=float(line[3])
                            
    
                    
                    
            current_line+=1
            
        
        for i in range(0,self.n_edges):
            current_action=self.actions[i]
            nodes=current_action.split('goto')[1].split('p')
            from_node=int(nodes[0])
            to_node=int(nodes[1])
            if not self.transitions[from_node][i][0][1]==1:
                self.transitions[from_node+self.n_nodes+self.n_edges][:]=self.transitions[from_node][:]
                self.transitions[from_node+self.n_nodes+self.n_edges][i]=False
                self.rewards[from_node+self.n_nodes+self.n_edges][:]=self.rewards[from_node][:]
                self.rewards[from_node+self.n_nodes+self.n_edges][i]=0
                self.prop_map[from_node+self.n_nodes+self.n_edges][from_node]=True 
        
                

        #print self.rewards        
        #print self.prop_map
        print self.transitions
        
        
        
        f.close()
        
        
        
        
    



class ProductMDP(MDP):

    def __init__(self, original_mdp,product_sta,product_lab,product_tra):
        
        #self.n_actions=
        #self.actions=
        
        #self.n_states=
        #self.state_labels=
        #self.goal_states=
        
        #self.rewards=
        
        #self.transitions=
        
        self.read_states(product_sta,product_lab)
        
        self.read_actions(product_tra)
        
        self.read_transitions(product_tra)
        
        self.read_rewards(original_mdp)
    
    def read_rewards(self,original_mdp):
        
        f = open(original_mdp, 'r')
         
        for line in f:
            #print(line)
            #print('rewards "time"\n')
            if line.startswith('rewards'):
                break
                
        self.rewards=[[0]*self.n_actions for i in range(self.n_states)]
        
        for line in f:
            if line.startswith('endrewards'):
                break
            line=line.split()
            if line !=[]:
                action=self.actions.index(line[0].strip('[]'))
                rest=line[1].split('=')[1].split(':')
                state=int(rest[0])
                reward=float(rest[1].rstrip(';'))
                for i in range(0,self.n_states):
                    if i not in self.goal_states:
                        if self.state_labels[i][1]==state:
                            if self.transitions[i][action]:
                                self.rewards[i][action]=reward
                
        f.close()
        
       
        
        
    
    def read_transitions(self,product_tra):
        f = open(product_tra, 'r')
        f.readline()
        
        self.transitions=[[False]*self.n_actions for i in range(self.n_states)]
        
        for line in f:
            line=line.split(' ')
            from_state=int(line[0])
            #line[1]=int(line[1])
            to_state=int(line[2])
            probability=float(line[3])
            action=self.actions.index(line[4].rstrip('\n'))
            if not self.transitions[from_state][action]:
                self.transitions[from_state][action]= [[to_state,probability]]
            else:
                self.transitions[from_state][action].append([to_state,probability])

        f.close()        
            
    
    def read_actions(self,product_tra):
        f = open(product_tra, 'r')
        f.readline()
        
        self.actions=[]
        self.n_actions=0
        
        for line in f:
            current_action=line.split(' ')[-1].rstrip('\n')
            if current_action not in self.actions:
                self.actions.append(current_action)
                self.n_actions=self.n_actions+1
                
        f.close()                
    
    
    
    
    def read_states(self,product_sta,product_lab):
        f = open(product_sta, 'r')
        f.readline()
        
        self.n_states=0
        self.state_labels=[]
        
        
        for line in f:
            current_state_label=line.split(':')[1]
            current_state_label=current_state_label.replace(')', '')
            current_state_label=current_state_label.replace('(', '')
            current_state_label=current_state_label.split(',')
            current_state_label[0]=int(current_state_label[0])
            current_state_label[1]=int(current_state_label[1])
            
            self.state_labels.append(current_state_label)
            
            
            
            self.n_states=self.n_states+1
            
        f.close()
        
        
        
        f = open(product_lab, 'r')
        
        line=f.readline()
        
        init_index=int(line.split('="init"')[0])
        
        target_index=line.split('="target"')[0]
        target_index=int(target_index.split(' ')[-1])
        
        
        
        
        self.goal_states=[]
        for line in f:
            int_line=[int(line.split(':')[0]),int(line.split(':')[1])]
            if int_line[1]==target_index:
                self.goal_states.append(int_line[0])
            if int_line[1]==init_index:
                self.initial_state=int_line[0]
                print(self.initial_state)
        
        f.close()
        
    


            
            
if __name__ == '__main__':
    
    graph=sys.argv[1]
    mdp=sys.argv[2]

    
    
    a=NavMDP(graph)
    
    a.write_prism_model(mdp)
    
    